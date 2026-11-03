#!/usr/bin/env python3

import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from maze_grid_node import MazeGrid


def edge_key(a, b):
    i, j = a
    di, dj = b[0]-i, b[1]-j
    return ('v', i if di > 0 else i-1, j) if di else ('h', i, j if dj > 0 else j-1)


def neighbours(cell):
    x, y = cell
    return [(x+1, y), (x, y+1), (x-1, y), (x, y-1)]


def reachable_cells(start, edges):
    parents = {start: None}
    queue = deque([start])
    while queue:
        a = queue.popleft()
        for b in neighbours(a):
            if b not in parents and edges.get(edge_key(a, b)) == 'open':
                parents[b] = a
                queue.append(b)
    return parents


def extend_corridor(cell, parent, edges, limit):
    if parent is None:
        return cell
    direction = cell[0]-parent[0], cell[1]-parent[1]
    for _ in range(max(0, limit-1)):
        exits = [b for b in neighbours(cell) if edges.get(edge_key(cell, b)) == 'open']
        ahead = cell[0]+direction[0], cell[1]+direction[1]
        if len(exits) != 2 or ahead not in exits:
            break
        cell = ahead
    return cell


def path_metrics(path, position, yaw):
    previous = position
    length, turn = 0.0, None
    for stamped in path.poses:
        p = stamped.pose.position
        point = p.x, p.y
        length += math.dist(previous, point)
        if turn is None and math.dist(position, point) >= 0.3:
            angle = math.atan2(point[1]-position[1], point[0]-position[0])-yaw
            turn = abs(math.atan2(math.sin(angle), math.cos(angle)))
        previous = point
    return length, turn or 0.0


class CellExplorer(MazeGrid):
    def __init__(self):
        super().__init__()
        defaults = dict(update_period=1.0, candidate_count=5, path_timeout=2.0,
                        path_weight=1.0, turn_weight=1.0, switch_margin=1.0,
                        corridor_cells=4, extension_distance=1.5, clearance=0.25,
                        retry_delay=10.0)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.visited = set()
        self.inspected = set()
        self.retry_after = {}
        self.active = None
        self.nav_handle = None
        self.nav_generation = 0
        self.evaluation = None
        self.request_id = 0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.planner = ActionClient(self, ComputePathToPose, '/exploration/compute_path_to_pose')
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.markers = self.create_publisher(MarkerArray, '/cell_explorer/markers', 10)
        self.create_timer(self.param('update_period'), self.tick)

    def param(self, name):
        return self.get_parameter(name).value

    def on_map(self, msg):
        super().on_map(msg)
        free = (self.cells == 0).astype(np.uint8)
        padded = np.pad(free, 1)
        self.clearances = cv2.distanceTransform(padded, cv2.DIST_L2, 5)[1:-1, 1:-1]*self.resolution

    def free_point(self, cell, probe=None):
        yy, xx = np.nonzero(self.clearances >= self.param('clearance'))
        world = (np.column_stack((xx, yy))+0.5)*self.resolution @ self.map_rotation.T+self.map_origin
        local = (world-self.grid_origin) @ self.grid_rotation
        center = 2*np.asarray(cell)
        inside = np.max(np.abs(local-center), axis=1) < (0.55 if probe is None else 0.75)
        if not np.any(inside):
            return None
        desired = center if probe is None else center+0.65*(np.asarray(probe)-np.asarray(cell))
        score = np.linalg.norm(local[inside]-desired, axis=1)
        best = world[inside][np.argmin(score)]
        return float(best[0]), float(best[1])

    def tick(self):
        if self.evaluation is not None:
            if time.monotonic()-self.request_started > self.param('path_timeout'):
                self.request_id += 1
                self.request_next()
            return
        if not self.fitted or not self.planner.server_is_ready() or not self.navigator.server_is_ready():
            return
        try:
            t = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
        except TransformException:
            return
        self.position = t.translation.x, t.translation.y
        q = t.rotation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        local = (np.asarray(self.position)-self.grid_origin) @ self.grid_rotation
        current = tuple(np.floor(local/2+0.5).astype(int))
        if np.max(np.abs(local-2*np.asarray(current))) <= 0.8:
            if current not in self.visited:
                self.visited.add(current)
                self.get_logger().info(f'Посещена клетка {current}; всего {len(self.visited)}')
        parents = reachable_cells(current, self.edges)
        steps = {current: 0}
        for cell, parent in parents.items():
            if parent is not None:
                steps[cell] = steps[parent]+1
        choices = {}
        for cell, parent in parents.items():
            if cell not in self.visited:
                end = extend_corridor(cell, parent, self.edges, self.param('corridor_cells'))
                choices[(end, None)] = self.free_point(end)
        choices = {k: p for k, p in choices.items() if p is not None}
        if self.active is not None and self.active['probe'] is None:
            intermediate = set()
            cell = parents.get(self.active['cell'])
            while cell is not None:
                intermediate.add(cell)
                cell = parents[cell]
            choices = {k: p for k, p in choices.items() if k[0] not in intermediate}
        if not choices and (self.active is None or self.active['probe'] is not None or
                            self.active['cell'] in self.visited):
            for cell in parents:
                for adjacent in neighbours(cell):
                    key = cell, adjacent
                    if self.edges.get(edge_key(cell, adjacent), 'unknown') == 'unknown' and key not in self.inspected:
                        choices[key] = self.free_point(cell, adjacent)
        candidates = [dict(cell=k[0], probe=k[1], point=p, key=k)
                      for k, p in choices.items() if p is not None and
                      self.retry_after.get(k, 0) <= time.monotonic()]
        candidates.sort(key=lambda c: steps[c['cell']])
        candidates = candidates[:self.param('candidate_count')]
        if self.active is not None:
            candidates = [self.active.copy()]+[c for c in candidates if c['key'] != self.active['key']]
        if not candidates:
            unresolved = sum(self.edges.get(edge_key(a, b), 'unknown') == 'unknown'
                             for a in parents for b in neighbours(a))
            self.get_logger().info(
                ('Достижимые клетки посещены, выходы проверены' if unresolved == 0 and
                 all(c in self.visited for c in parents) else
                 f'Нет доступных целей; достижимо клеток {len(parents)}, неизвестных переходов {unresolved}'),
                throttle_duration_sec=10.0)
            self.publish_cells(None)
            return
        self.evaluation = dict(queue=deque(candidates), valid=[], position=self.position, yaw=self.yaw)
        self.request_next()

    def pose_message(self, point):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = point
        pose.pose.orientation.w = 1.0
        return pose

    def request_next(self):
        if not self.evaluation['queue']:
            self.finish_evaluation()
            return
        candidate = self.evaluation['queue'].popleft()
        self.request_id += 1
        request = self.request_id
        self.request_started = time.monotonic()
        goal = ComputePathToPose.Goal()
        goal.goal = self.pose_message(candidate['point'])
        goal.planner_id = 'GridBased'
        future = self.planner.send_goal_async(goal)

        def accepted(f):
            handle = f.result()
            if request != self.request_id:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.request_next()
                return
            handle.get_result_async().add_done_callback(done)

        def done(f):
            if request != self.request_id:
                return
            result = f.result()
            if result.status == 4 and result.result.path.poses:
                length, turn = path_metrics(result.result.path, self.evaluation['position'], self.evaluation['yaw'])
                candidate['cost'] = self.param('path_weight')*length+self.param('turn_weight')*turn
                candidate['length'] = length
                self.evaluation['valid'].append(candidate)
            else:
                self.retry_after[candidate['key']] = time.monotonic()+self.param('retry_delay')
            self.request_next()

        future.add_done_callback(accepted)

    def finish_evaluation(self):
        valid = [c for c in self.evaluation['valid']
                 if (c['probe'] is None and (c['cell'] not in self.visited or
                     self.active and c['key'] == self.active['key'])) or
                 (c['probe'] is not None and c['key'] not in self.inspected)]
        self.evaluation = None
        if not valid:
            return
        best = min(valid, key=lambda c: c['cost'])
        current = next((c for c in valid if self.active and c['key'] == self.active['key']), None)
        if current:
            direction = np.asarray(current['point'])-self.position
            extensions = [c for c in valid if c['probe'] is None and c['key'] != current['key']
                          and (c['cell'][0] == current['cell'][0] or c['cell'][1] == current['cell'][1])
                          and np.dot(np.asarray(c['point'])-current['point'], direction) > 0.5]
            if current['length'] > self.param('extension_distance'):
                if best['cost'] >= current['cost']-self.param('switch_margin'):
                    best = current
            elif extensions:
                best = min(extensions, key=lambda c: c['cost'])
            elif current['cell'] in self.visited:
                onward = [c for c in valid if c['key'] != current['key']]
                if onward:
                    best = min(onward, key=lambda c: c['cost'])
            if best['key'] == current['key']:
                self.publish_cells(current['point'])
                return
        self.send_navigation(best)

    def send_navigation(self, candidate):
        self.nav_generation += 1
        generation = self.nav_generation
        self.active = candidate
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_message(candidate['point'])
        self.get_logger().info(
            f'Цель {candidate["cell"]}, путь {candidate["length"]:.2f} м, оценка {candidate["cost"]:.2f}'
            + (' — осмотр перехода' if candidate['probe'] else ''))

        def accepted(f):
            handle = f.result()
            if generation != self.nav_generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.active = None
                return
            self.nav_handle = handle
            handle.get_result_async().add_done_callback(done)

        def done(f):
            if generation != self.nav_generation:
                return
            if f.result().status == 4 and candidate['probe']:
                self.inspected.add(candidate['key'])
            elif f.result().status != 4:
                self.retry_after[candidate['key']] = time.monotonic()+self.param('retry_delay')
            self.active = self.nav_handle = None

        self.navigator.send_goal_async(goal).add_done_callback(accepted)
        self.publish_cells(candidate['point'])

    def publish_cells(self, goal):
        marker = Marker()
        marker.header.frame_id = 'maze_grid'
        marker.ns = 'visited_cells'
        marker.type = Marker.CUBE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 1.8
        marker.scale.z = 0.01
        marker.color.b, marker.color.g, marker.color.a = 1.0, 0.5, 0.15
        marker.points = [Point(x=float(2*x), y=float(2*y), z=0.02) for x, y in sorted(self.visited)]
        marker.colors = [ColorRGBA(r=0.0 if all(self.edges.get(edge_key(c, b), 'unknown') != 'unknown'
                            for b in neighbours(c)) else 1.0, g=0.8, b=0.2, a=0.15)
                         for c in sorted(self.visited)]
        target = Marker()
        target.header.frame_id = 'map'
        target.ns = 'cell_goal'
        target.type = Marker.SPHERE
        target.pose.orientation.w = 1.0
        target.scale.x = target.scale.y = target.scale.z = 0.18
        target.color.r = target.color.a = 1.0
        if goal is not None:
            target.pose.position = Point(x=goal[0], y=goal[1], z=0.15)
        else:
            target.action = Marker.DELETE
        self.markers.publish(MarkerArray(markers=[marker, target]))


def main():
    rclpy.init()
    node = CellExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.nav_handle is not None and rclpy.ok():
            node.nav_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
