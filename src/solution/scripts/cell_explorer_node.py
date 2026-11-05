#!/usr/bin/env python3

import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from tf2_geometry_msgs import do_transform_point
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformException, TransformListener
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from maze_grid_node import MazeGrid


def edge_key(a, b):
    i, j = a
    di, dj = b[0] - i, b[1] - j
    return ("v", i if di > 0 else i - 1, j) if di else ("h", i, j if dj > 0 else j - 1)


def neighbours(cell):
    x, y = cell
    return [(x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)]


def reachable_cells(start, edges):
    parents = {start: None}
    queue = deque([start])
    while queue:
        a = queue.popleft()
        for b in neighbours(a):
            if b not in parents and edges.get(edge_key(a, b)) == "open":
                parents[b] = a
                queue.append(b)
    return parents


def extend_corridor(cell, parent, edges, limit):
    if parent is None:
        return cell
    direction = cell[0] - parent[0], cell[1] - parent[1]
    for _ in range(max(0, limit - 1)):
        exits = [b for b in neighbours(cell) if edges.get(edge_key(cell, b)) == "open"]
        ahead = cell[0] + direction[0], cell[1] + direction[1]
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
            angle = math.atan2(point[1] - position[1], point[0] - position[0]) - yaw
            turn = abs(math.atan2(math.sin(angle), math.cos(angle)))
        previous = point
    return length, turn or 0.0


def perimeter_hypotheses(known, start, size):
    low_x, high_x = min(c[0] for c in known), max(c[0] for c in known)
    low_y, high_y = min(c[1] for c in known), max(c[1] for c in known)
    boxes = []
    for x in range(start[0] - size + 1, start[0] + 1):
        for y in range(start[1] - size + 1, start[1] + 1):
            contains_known = (
                x <= low_x and high_x < x + size and y <= low_y and high_y < y + size
            )
            start_on_boundary = start[0] in (x, x + size - 1) or start[1] in (
                y,
                y + size - 1,
            )
            if contains_known and start_on_boundary:
                boxes.append((x, y))
    return boxes


def perimeter_score(cell, boxes, size):
    if not boxes:
        return 0.0
    boundary_count = sum(
        cell[0] in (x, x + size - 1) or cell[1] in (y, y + size - 1) for x, y in boxes
    )
    return boundary_count / len(boxes)


class CellExplorer(MazeGrid):
    def __init__(self):
        super().__init__()
        defaults = dict(
            update_period=1.0,
            candidate_count=5,
            path_timeout=2.0,
            path_weight=1.0,
            turn_weight=2.0,
            switch_margin=1.0,
            corridor_cells=4,
            extension_distance=1.5,
            clearance=0.25,
            retry_delay=10.0,
            maze_cells=8,
            perimeter_weight=4.0,
        )
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.visited = set()
        self.uninteresting = set()
        self.start_cell = None
        self.perimeter_boxes = []
        self.inspected = set()
        self.retry_after = {}
        self.active = None
        self.nav_handle = None
        self.nav_generation = 0
        self.evaluation = None
        self.request_id = 0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.planner = ActionClient(
            self, ComputePathToPose, "/exploration/compute_path_to_pose"
        )
        self.navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.markers = self.create_publisher(MarkerArray, "/cell_explorer/markers", 10)
        self.delivery_state = "EXPLORE"
        self.start_point = None
        self.station_cell = None
        self.station_result = self.create_publisher(
            String, "/delivery/navigation_result", 10
        )
        self.create_subscription(
            PointStamped, "/delivery/station_target", self.on_station_target, 10
        )
        self.create_subscription(String, "/delivery/state", self.on_delivery_state, 10)
        self.create_timer(self.param("update_period"), self.tick)

    def on_delivery_state(self, msg: String):
        if msg.data == self.delivery_state:
            return
        self.get_logger().info(f"Доставка: {self.delivery_state} -> {msg.data}")
        self.delivery_state = msg.data
        self.station_cell = None
        self.evaluation = None
        self.request_id += 1
        self.nav_generation += 1
        if self.nav_handle is not None:
            self.nav_handle.cancel_goal_async()
        self.nav_handle = None
        self.active = None

    def on_station_target(self, msg):
        if (
            self.delivery_state != "DOCK_STATION"
            or self.station_cell is not None
            or not self.fitted
        ):
            return
        try:
            transform = self.buffer.lookup_transform(
                "map", msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp)
            )
        except TransformException:
            return
        point = do_transform_point(msg, transform).point
        local = (np.array([point.x, point.y]) - self.grid_origin) @ self.grid_rotation
        self.station_cell = tuple(np.floor(local / 2 + 0.5).astype(int))
        self.get_logger().info(f"Клетка станции: {self.station_cell}")

    def param(self, name):
        return self.get_parameter(name).value

    def on_map(self, msg):
        super().on_map(msg)
        self.inspected = {
            key
            for key in self.inspected
            if self.edges.get(edge_key(*key)) in ("open", "wall")
        }
        free = (self.cells == 0).astype(np.uint8)
        padded = np.pad(free, 1)
        self.clearances = (
            cv2.distanceTransform(padded, cv2.DIST_L2, 5)[1:-1, 1:-1] * self.resolution
        )
        self.refresh_uninteresting()
        self.publish_cells(self.active["point"] if self.active else None)

    def is_uninteresting(self, cell):
        size = self.param("maze_cells")
        return (
            bool(self.perimeter_boxes)
            and all(
                x < cell[0] < x + size - 1 and y < cell[1] < y + size - 1
                for x, y in self.perimeter_boxes
            )
            and all(
                self.edges.get(edge_key(cell, adjacent)) in ("open", "wall")
                for adjacent in neighbours(cell)
            )
        )

    def refresh_uninteresting(self):
        cells = set()
        for axis, i, j in self.edges:
            cells.add((i, j))
            cells.add((i + 1, j) if axis == "v" else (i, j + 1))
        self.uninteresting = {cell for cell in cells if self.is_uninteresting(cell)}
        if (
            self.delivery_state == "EXPLORE"
            and self.active is not None
            and self.active["probe"] is None
            and self.active["cell"] in self.uninteresting
        ):
            self.get_logger().info(
                f'Клетка {self.active["cell"]} больше не требует посещения; перевыбор цели.'
            )
            self.evaluation = None
            self.request_id += 1
            self.nav_generation += 1
            if self.nav_handle is not None:
                self.nav_handle.cancel_goal_async()
            self.active = self.nav_handle = None

    def free_point(self, cell, probe=None):
        req_clearance = self.param("clearance")
        center = 2 * np.asarray(cell)
        limit = 0.55 if probe is None else 0.75
        for cl in (req_clearance, 0.18, 0.14, 0.08):
            yy, xx = np.nonzero(self.clearances >= cl)
            if len(xx) == 0:
                continue
            world = (
                np.column_stack((xx, yy)) + 0.5
            ) * self.resolution @ self.map_rotation.T + self.map_origin
            local = (world - self.grid_origin) @ self.grid_rotation
            inside = np.max(np.abs(local - center), axis=1) < limit
            if np.any(inside):
                desired = (
                    center
                    if probe is None
                    else center + 0.65 * (np.asarray(probe) - np.asarray(cell))
                )
                score = np.linalg.norm(local[inside] - desired, axis=1)
                best = world[inside][np.argmin(score)]
                return float(best[0]), float(best[1])
        return None

    def tick(self):
        if self.delivery_state in (
            "LOAD_CARGO",
            "DOCK_UNLOAD",
            "UNLOAD",
            "MISSION_COMPLETE",
        ):
            if self.nav_handle is not None:
                self.nav_handle.cancel_goal_async()
                self.nav_handle = None
                self.active = None
            return

        if self.evaluation is not None:
            if time.monotonic() - self.request_started > self.param("path_timeout"):
                self.request_id += 1
                self.request_next()
            return
        if not self.fitted:
            return
        if not self.planner.server_is_ready():
            self.get_logger().warn(
                "Ожидание сервера планировщика /exploration/compute_path_to_pose...",
                throttle_duration_sec=5.0,
            )
            return
        if not self.navigator.server_is_ready():
            self.get_logger().warn(
                "Ожидание сервера навигатора /navigate_to_pose...",
                throttle_duration_sec=5.0,
            )
            return
        try:
            t = self.buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            ).transform
        except TransformException:
            return
        self.position = t.translation.x, t.translation.y
        q = t.rotation
        self.yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
        )
        if self.start_point is None:
            self.start_point = (self.position[0], self.position[1])

        if self.delivery_state == "DOCK_STATION":
            if self.station_cell is None or self.active is not None:
                return
            center_grid = 2.0 * np.asarray(self.station_cell)
            center_world = center_grid @ self.grid_rotation.T + self.grid_origin
            point = (float(center_world[0]), float(center_world[1]))
            self.send_navigation(
                dict(
                    cell=self.station_cell,
                    probe=None,
                    point=point,
                    key=(self.station_cell, "station"),
                    cost=0.0,
                    length=math.dist(self.position, point),
                    is_station=True,
                )
            )
            return

        if self.delivery_state == "NAV_TO_UNLOAD":
            if self.active is not None and self.active.get("is_home"):
                return
            target_point = None
            if self.start_point is not None:
                target_point = self.start_point
            elif self.start_cell is not None:
                target_point = self.free_point(self.start_cell)
            if target_point is not None:
                home_candidate = dict(
                    cell=self.start_cell if self.start_cell is not None else (0, 0),
                    probe=None,
                    point=target_point,
                    key=(self.start_cell, "home"),
                    cost=0.0,
                    length=math.dist(self.position, target_point),
                    perimeter=0.0,
                    is_home=True,
                )
                self.get_logger().info(
                    f"[Explorer] Навигация на базу выгрузки: старт {self.start_cell}, цель {target_point}"
                )
                self.send_navigation(home_candidate)
            return

        local = (np.asarray(self.position) - self.grid_origin) @ self.grid_rotation
        current = tuple(np.floor(local / 2 + 0.5).astype(int))
        if np.max(np.abs(local - 2 * np.asarray(current))) <= 0.8:
            if current not in self.visited:
                if self.start_cell is None:
                    self.start_cell = current
                self.visited.add(current)
                self.get_logger().info(
                    f"Посещена клетка {current}; всего {len(self.visited)}"
                )
        parents = reachable_cells(current, self.edges)
        if self.start_cell is not None:
            self.perimeter_boxes = perimeter_hypotheses(
                set(parents) | self.visited, self.start_cell, self.param("maze_cells")
            )
        self.refresh_uninteresting()
        steps = {current: 0}
        for cell, parent in parents.items():
            if parent is not None:
                steps[cell] = steps[parent] + 1
        choices = {}
        for cell, parent in parents.items():
            if cell not in self.visited and cell not in self.uninteresting:
                end = extend_corridor(
                    cell, parent, self.edges, self.param("corridor_cells")
                )
                if end in self.uninteresting:
                    end = cell
                choices[(end, None)] = self.free_point(end)
        choices = {k: p for k, p in choices.items() if p is not None}
        if self.active is not None and self.active["probe"] is None:
            intermediate = set()
            cell = parents.get(self.active["cell"])
            while cell is not None:
                intermediate.add(cell)
                cell = parents[cell]
            choices = {k: p for k, p in choices.items() if k[0] not in intermediate}
        for cell in parents:
            for adjacent in neighbours(cell):
                key = cell, adjacent
                if (
                    self.edges.get(edge_key(cell, adjacent), "unknown") == "unknown"
                    and key not in self.inspected
                ):
                    choices[key] = self.free_point(cell, adjacent)
        candidates = [
            dict(cell=k[0], probe=k[1], point=p, key=k)
            for k, p in choices.items()
            if p is not None and self.retry_after.get(k, 0) <= time.monotonic()
        ]
        candidates.sort(
            key=lambda c: 2 * steps[c["cell"]]
            - self.param("perimeter_weight") * self.perimeter_bonus(c)
        )
        candidates = candidates[: self.param("candidate_count")]
        if self.active is not None:
            candidates = [self.active.copy()] + [
                c for c in candidates if c["key"] != self.active["key"]
            ]
        if not candidates:
            unresolved = sum(
                self.edges.get(edge_key(a, b), "unknown") == "unknown"
                for a in parents
                for b in neighbours(a)
            )
            if unresolved > 0:
                self.get_logger().info(
                    f"Сброс inspected/retry_after для повторной проверки {unresolved} неизвестных переходов..."
                )
                self.inspected.clear()
                self.retry_after.clear()
            self.get_logger().info(
                (
                    "Достижимые клетки посещены, выходы проверены"
                    if unresolved == 0 and all(c in self.visited for c in parents)
                    else f"Нет доступных целей; достижимо клеток {len(parents)}, неизвестных переходов {unresolved}"
                ),
                throttle_duration_sec=10.0,
            )
            self.publish_cells(None)
            return
        self.evaluation = dict(
            queue=deque(candidates), valid=[], position=self.position, yaw=self.yaw
        )
        self.request_next()

    def pose_message(self, point):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x, pose.pose.position.y = point
        pose.pose.orientation.w = 1.0
        return pose

    def perimeter_bonus(self, candidate):
        if candidate["probe"] is not None or candidate["cell"] in self.visited:
            return 0.0
        return perimeter_score(
            candidate["cell"], self.perimeter_boxes, self.param("maze_cells")
        )

    def request_next(self):
        if not self.evaluation["queue"]:
            self.finish_evaluation()
            return
        candidate = self.evaluation["queue"].popleft()
        self.request_id += 1
        request = self.request_id
        self.request_started = time.monotonic()
        goal = ComputePathToPose.Goal()
        goal.goal = self.pose_message(candidate["point"])
        goal.planner_id = "GridBased"
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
                length, turn = path_metrics(
                    result.result.path,
                    self.evaluation["position"],
                    self.evaluation["yaw"],
                )
                candidate["perimeter"] = self.perimeter_bonus(candidate)
                candidate["cost"] = (
                    self.param("path_weight") * length
                    + self.param("turn_weight") * turn
                    - self.param("perimeter_weight") * candidate["perimeter"]
                )
                candidate["length"] = length
                self.evaluation["valid"].append(candidate)
            else:
                self.retry_after[candidate["key"]] = time.monotonic() + self.param(
                    "retry_delay"
                )
            self.request_next()

        future.add_done_callback(accepted)

    def finish_evaluation(self):
        valid = []
        active_key = self.active["key"] if self.active else None
        for candidate in self.evaluation["valid"]:
            if candidate["probe"] is None:
                needed = (
                    candidate["cell"] not in self.visited
                    or candidate["key"] == active_key
                ) and candidate["cell"] not in self.uninteresting
            else:
                needed = candidate["key"] not in self.inspected
            if needed:
                valid.append(candidate)
        self.evaluation = None
        if not valid:
            return
        best = min(valid, key=lambda c: c["cost"])
        current = next(
            (c for c in valid if self.active and c["key"] == self.active["key"]), None
        )
        if current:
            direction = np.asarray(current["point"]) - self.position
            extensions = []
            for candidate in valid:
                if candidate["probe"] is not None or candidate["key"] == current["key"]:
                    continue
                same_axis = (
                    candidate["cell"][0] == current["cell"][0]
                    or candidate["cell"][1] == current["cell"][1]
                )
                ahead = (
                    np.dot(np.asarray(candidate["point"]) - current["point"], direction)
                    > 0.5
                )
                if same_axis and ahead:
                    extensions.append(candidate)
            if current["length"] > self.param("extension_distance"):
                if best["cost"] >= current["cost"] - self.param("switch_margin"):
                    best = current
            elif extensions and best["probe"] is None:
                best = min(extensions, key=lambda c: c["cost"])
            elif current["probe"] is None and current["cell"] not in self.visited:
                best = current
            elif current["probe"] is None and current["cell"] in self.visited:
                onward = [c for c in valid if c["key"] != current["key"]]
                if onward:
                    best = min(onward, key=lambda c: c["cost"])
            if best["key"] == current["key"]:
                self.publish_cells(current["point"])
                return
        self.send_navigation(best)

    def send_navigation(self, candidate):
        self.nav_generation += 1
        generation = self.nav_generation
        self.active = candidate
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_message(candidate["point"])
        self.get_logger().info(
            f'Цель {candidate["cell"]}, путь {candidate["length"]:.2f} м, оценка {candidate["cost"]:.2f}, '
            f'периметр {candidate.get("perimeter", 0):.0%}'
            + (" — осмотр перехода" if candidate["probe"] else "")
        )

        def accepted(f):
            handle = f.result()
            if generation != self.nav_generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.get_logger().warn(f'Nav2 отклонил цель {candidate["cell"]}.')
                self.active = None
                if candidate.get("is_station"):
                    self.station_result.publish(String(data="failed"))
                return
            self.get_logger().info(f'Nav2 принял цель {candidate["cell"]}.')
            self.nav_handle = handle
            handle.get_result_async().add_done_callback(done)

        def done(f):
            if generation != self.nav_generation:
                return
            status = f.result().status
            label = {4: "достигнута", 5: "отменена", 6: "ошибка"}.get(
                status, str(status)
            )
            self.get_logger().info(f'Nav2: цель {candidate["cell"]} — {label}.')
            if candidate.get("is_station") or candidate.get("is_home"):
                self.station_result.publish(
                    String(data="reached" if status == 4 else "failed")
                )
            if status == 4 and candidate["probe"]:
                edge = edge_key(candidate["cell"], candidate["probe"])
                if self.edges.get(edge) in ("open", "wall"):
                    self.inspected.add(candidate["key"])
                else:
                    self.retry_after[candidate["key"]] = time.monotonic() + self.param(
                        "retry_delay"
                    )
                    self.get_logger().warn(
                        f'Переход {candidate["key"]} остался неизвестным; повторный осмотр через '
                        f'{self.param("retry_delay"):.0f} с.'
                    )
            elif status != 4:
                self.retry_after[candidate["key"]] = time.monotonic() + self.param(
                    "retry_delay"
                )
            self.active = self.nav_handle = None

        self.navigator.send_goal_async(goal).add_done_callback(accepted)
        self.publish_cells(candidate["point"])

    def publish_cells(self, goal):
        marker = Marker()
        marker.header.frame_id = "maze_grid"
        marker.ns = "visited_cells"
        marker.type = Marker.CUBE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 1.8
        marker.scale.z = 0.01
        marker.color.b, marker.color.g, marker.color.a = 1.0, 0.5, 0.15
        for cell in sorted(self.visited | self.uninteresting):
            marker.points.append(
                Point(x=float(2 * cell[0]), y=float(2 * cell[1]), z=0.02)
            )
            exits_known = all(
                self.edges.get(edge_key(cell, b), "unknown") != "unknown"
                for b in neighbours(cell)
            )
            if cell not in self.visited:
                marker.colors.append(ColorRGBA(r=0.45, g=0.45, b=0.45, a=0.45))
            else:
                marker.colors.append(
                    ColorRGBA(r=0.0 if exits_known else 1.0, g=0.8, b=0.2, a=0.15)
                )
        target = Marker()
        target.header.frame_id = "map"
        target.ns = "cell_goal"
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


if __name__ == "__main__":
    main()
