#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from maze_grid_node import MazeGrid


class Tremaux:
    def __init__(self):
        self.current = None
        self.seen = set()
        self.path = []
        self.marks = {}
        self.loop_return = None

    @staticmethod
    def edge(a, b):
        return tuple(sorted((a, b)))

    def arrive(self, node):
        if node == self.current:
            return
        if self.current is not None:
            edge = self.edge(self.current, node)
            self.marks[edge] = self.marks.get(edge, 0)+1
        if node == self.loop_return:
            self.loop_return = None
        elif node not in self.seen:
            self.seen.add(node)
            self.path.append(node)
        elif len(self.path) > 1 and node == self.path[-2]:
            self.path.pop()
        else:
            self.loop_return = self.current
        self.current = node

    def choose(self, exits):
        if self.loop_return is not None:
            return self.loop_return if self.loop_return in exits else None
        for node in exits:
            if self.marks.get(self.edge(self.current, node), 0) == 0:
                return node
        if len(self.path) > 1 and self.path[-2] in exits:
            return self.path[-2]
        return None


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TremauxExplorer(MazeGrid):
    def __init__(self):
        super().__init__()
        self.graph = Tremaux()
        self.target = None
        self.target_time = None
        self.scan = None
        self.scan_time = None
        self.avoid = None
        self.avoid_start = None
        self.finished = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.graph_pub = self.create_publisher(MarkerArray, '/tremaux/graph', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.1, self.control_loop)

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = self.get_clock().now()

    def edge_state(self, a, b):
        i, j = a
        di, dj = b[0]-i, b[1]-j
        key = ('v', i if di > 0 else i-1, j) if di else ('h', i, j if dj > 0 else j-1)
        return self.edges.get(key, 'unknown')

    def control_loop(self):
        now = self.get_clock().now()
        if (self.finished or self.scan is None or not self.fitted or
                (now-self.scan_time).nanoseconds > 500_000_000):
            self.drive(0, 0)
            return
        try:
            pose = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
        except TransformException:
            self.drive(0, 0)
            return
        x, y = pose.translation.x, pose.translation.y
        q = pose.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        local = (np.array([x, y])-self.grid_origin) @ self.grid_rotation
        cell = tuple(np.floor(local/2+0.5).astype(int))
        if np.max(np.abs(local-2*np.array(cell))) <= 0.8 and cell != self.graph.current:
            previous = self.graph.current
            if previous is not None and sum(abs(a-b) for a, b in zip(previous, cell)) != 1:
                self.get_logger().error('Положение перескочило через клетку; остановка без объявления завершения')
                self.finished = True
                self.drive(0, 0)
                return
            if previous is not None and cell != self.target:
                self.get_logger().error('Объезд привёл в другую клетку; не считаем это завершением ветви Тремо')
                self.finished = True
                self.drive(0, 0)
                return
            self.graph.arrive(cell)
            self.target = None
            self.avoid = None
            self.get_logger().info(f'Тремо: клетка {cell}, посещено {len(self.graph.seen)}')
        current = self.graph.current
        if current is None:
            self.drive(0, 0)
            return
        if self.target is not None and self.edge_state(current, self.target) == 'wall':
            self.target = None
            self.avoid = None
        if self.target is None:
            heading = round((yaw-self.angle)/(math.pi/2)) % 4
            directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
            exits = [(current[0]+directions[d][0], current[1]+directions[d][1])
                     for d in ((heading-1)%4, heading, (heading+1)%4, (heading+2)%4)]
            exits = [b for b in exits if self.edge_state(current, b) != 'wall']
            exits.sort(key=lambda b: self.edge_state(current, b) != 'open')
            self.target = self.graph.choose(exits)
            self.target_time = now
            if self.target is None:
                self.finished = True
                complete = not self.graph.path[1:] and self.graph.loop_return is None
                self.get_logger().info('Тремо: доступный граф обойдён; возврат к старту' if complete
                                       else 'Тремо: обратный проход не подтверждён; остановка без завершения')
                self.drive(0, 0)
                return
            self.get_logger().info(f'Тремо: переход {current} → {self.target}')
        if (now-self.target_time).nanoseconds > 45_000_000_000:
            self.get_logger().error('Не удалось пройти выбранный переход за 45 с; обход не завершён')
            self.finished = True
            self.drive(0, 0)
            return

        goal = np.array(self.target)*2 @ self.grid_rotation.T+self.grid_origin
        error = wrap(math.atan2(goal[1]-y, goal[0]-x)-yaw)
        front = self.sector_min(-25, 25)
        left, right = self.sector_min(30, 90), self.sector_min(-90, -30)
        if self.avoid is not None:
            travelled = math.hypot(x-self.avoid_start[0], y-self.avoid_start[1])
            if travelled > 0.4 and abs(error) < 1.0 and self.clearance(error) > 1.0:
                self.avoid = None
        if front < 0.45:
            if self.avoid is None:
                self.avoid = 1 if left > right else -1
                self.avoid_start = (x, y)
            self.drive(0, self.avoid*1.2)
        else:
            side = -0.8 if left < 0.3 else 0.8 if right < 0.3 else 0.0
            turn = side if self.avoid is not None else max(-1.2, min(1.2, 1.5*error+side))
            speed = 0.35 if self.avoid is not None else 0.35*max(0.0, math.cos(error))
            self.drive(speed, turn)
        self.publish_graph(goal)

    def sector_min(self, low, high):
        values = [r for i, r in enumerate(self.scan.ranges)
                  if low <= math.degrees(wrap(self.scan.angle_min+i*self.scan.angle_increment)) <= high
                  and r >= self.scan.range_min]
        return min(values, default=0.0)

    def clearance(self, heading):
        best = self.scan.range_max
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or r < self.scan.range_min:
                continue
            angle = self.scan.angle_min+i*self.scan.angle_increment-heading
            forward, side = r*math.cos(angle), abs(r*math.sin(angle))
            if forward > 0 and side < 0.27:
                best = min(best, max(0, forward-math.sqrt(0.27**2-side**2)))
        return best

    def drive(self, speed, turn):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(speed), float(turn)
        self.cmd_pub.publish(cmd)

    def publish_graph(self, goal):
        marker = Marker()
        marker.header.frame_id = 'maze_grid'
        marker.ns = 'traversed_passages'
        marker.type = Marker.LINE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.color.g = marker.color.a = 1.0
        for a, b in self.graph.marks:
            marker.points.extend([Point(x=float(2*a[0]), y=float(2*a[1]), z=0.1),
                                  Point(x=float(2*b[0]), y=float(2*b[1]), z=0.1)])
        target = Marker()
        target.header.frame_id = 'map'
        target.ns = 'tremaux_target'
        target.type = Marker.SPHERE
        target.pose.orientation.w = 1.0
        target.pose.position = Point(x=float(goal[0]), y=float(goal[1]), z=0.15)
        target.scale.x = target.scale.y = target.scale.z = 0.18
        target.color.r = target.color.a = 1.0
        self.graph_pub.publish(MarkerArray(markers=[marker, target]))


def main():
    rclpy.init()
    node = TremauxExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.drive(0, 0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
