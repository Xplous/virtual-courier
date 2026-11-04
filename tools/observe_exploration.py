#!/usr/bin/env python3
"""Independent evaluator; privileged scene data never enters the robot algorithm."""

import argparse
import json
import math
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Observer(Node):
    def __init__(self, folder):
        super().__init__("coverage_observer")
        self.folder = folder
        self.meta = json.loads((folder / "world.json").read_text())
        self.size = self.meta["cell_size"]
        self.cols, self.rows = self.meta["grid"]
        self.clock = 0.0
        self.state = None
        self.poses = deque(maxlen=40)
        self.scans = deque(maxlen=40)
        self.visited, self.traversed = set(), set()
        self.last_cell = None
        self.hits, self.free = {}, {}
        self.truth = {}
        self.claimed_complete = False
        self.first_full_cells = self.first_full_survey = None
        self.done = False
        self.last_write = 0.0
        self.samples = self.processed_scans = 0
        self.trajectory = (folder / "trajectory.jsonl").open("w")
        self.build_truth()
        self.create_subscription(
            Clock, "/clock", self.on_clock, qos_profile_sensor_data
        )
        self.create_subscription(String, "/referee/state", self.on_state, 10)
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Log, "/rosout", self.on_log, 100)

    def build_truth(self):
        def wall_at(x, y):
            for (ax, ay), (bx, by) in self.meta["walls"]:
                if (
                    abs(ax - bx) < 1e-6
                    and abs(x - ax) < 1e-6
                    and min(ay, by) <= y <= max(ay, by)
                ):
                    return True
                if (
                    abs(ay - by) < 1e-6
                    and abs(y - ay) < 1e-6
                    and min(ax, bx) <= x <= max(ax, bx)
                ):
                    return True
            return False

        for i in range(self.cols + 1):
            for j in range(self.rows):
                self.truth[("v", i, j)] = wall_at(i * self.size, (j + 0.5) * self.size)
        for i in range(self.cols):
            for j in range(self.rows + 1):
                self.truth[("h", i, j)] = wall_at((i + 0.5) * self.size, j * self.size)
        self.reachable = {tuple(self.meta["start"]["cell"])}
        queue = deque(self.reachable)
        while queue:
            cell = queue.popleft()
            for other, edge in self.incident(cell):
                if not self.truth[edge] and other not in self.reachable:
                    self.reachable.add(other)
                    queue.append(other)
        self.required_edges = {
            edge for cell in self.reachable for _, edge in self.incident(cell)
        }
        for edge in self.required_edges:
            self.hits[edge] = np.zeros(16, dtype=bool)
            self.free[edge] = np.zeros(16, dtype=bool)

    @staticmethod
    def incident(cell):
        x, y = cell
        return [
            ((x + 1, y), ("v", x + 1, y)),
            ((x - 1, y), ("v", x, y)),
            ((x, y + 1), ("h", x, y + 1)),
            ((x, y - 1), ("h", x, y)),
        ]

    def on_clock(self, msg):
        self.clock = msg.clock.sec + msg.clock.nanosec / 1e9

    def on_log(self, msg):
        if (
            msg.name == "cell_explorer"
            and "Достижимые клетки посещены, выходы проверены" in msg.msg
        ):
            self.claimed_complete = True

    def on_state(self, msg):
        self.state = json.loads(msg.data)
        robot = self.state["robot"]
        x, y, yaw = robot["x"], robot["y"], robot["yaw"]
        self.poses.append((self.clock, x, y, yaw))
        self.samples += 1
        self.trajectory.write(
            json.dumps(
                dict(t=self.clock, elapsed=self.state["elapsed"], x=x, y=y, yaw=yaw)
            )
            + "\n"
        )
        cell = math.floor(x / self.size), math.floor(y / self.size)
        if cell in self.reachable:
            if (
                min(
                    x - cell[0] * self.size,
                    (cell[0] + 1) * self.size - x,
                    y - cell[1] * self.size,
                    (cell[1] + 1) * self.size - y,
                )
                >= 0.2
            ):
                self.visited.add(cell)
            if self.last_cell and cell != self.last_cell:
                for neighbour, edge in self.incident(self.last_cell):
                    if (
                        neighbour == cell
                        and edge in self.truth
                        and not self.truth[edge]
                    ):
                        self.traversed.add(edge)
            self.last_cell = cell
        self.process_scans()
        elapsed = self.state["elapsed"]
        checked = self.checked_edges()
        surveyed = {
            c for c in self.visited if all(e in checked for _, e in self.incident(c))
        }
        if len(self.visited) == len(self.reachable) and self.first_full_cells is None:
            self.first_full_cells = elapsed
        if len(surveyed) == len(self.reachable) and self.first_full_survey is None:
            self.first_full_survey = elapsed
        self.done = bool(
            self.state["finished"]
            or elapsed >= 600
            or self.first_full_survey is not None
        )
        if time.monotonic() - self.last_write > 5 or self.done:
            self.write_result(final=self.done)
            self.last_write = time.monotonic()

    def on_scan(self, msg):
        self.scans.append(msg)
        self.process_scans()

    def process_scans(self):
        while self.scans and len(self.poses) >= 2:
            scan = self.scans[0]
            stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
            if stamp > self.poses[-1][0]:
                return
            self.scans.popleft()
            if stamp < self.poses[0][0]:
                continue
            for a, b in zip(self.poses, list(self.poses)[1:]):
                if a[0] <= stamp <= b[0] and b[0] > a[0]:
                    f = (stamp - a[0]) / (b[0] - a[0])
                    yaw_delta = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
                    self.observe_scan(
                        scan,
                        a[1] + f * (b[1] - a[1]),
                        a[2] + f * (b[2] - a[2]),
                        a[3] + f * yaw_delta,
                    )
                    break

    def observe_scan(self, scan, x, y, yaw):
        self.processed_scans += 1
        angles = (
            scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment + yaw
        )
        ranges = np.minimum(np.asarray(scan.ranges), scan.range_max)
        valid_range = ~np.isnan(ranges) & (ranges >= scan.range_min)
        origin = np.array([x + 0.09 * math.cos(yaw), y + 0.09 * math.sin(yaw)])
        directions = np.stack((np.cos(angles), np.sin(angles)), axis=1)
        for axis, count in [(0, self.cols), (1, self.rows)]:
            for line in range(count + 1):
                divisor = directions[:, axis]
                t = np.divide(
                    line * self.size - origin[axis],
                    divisor,
                    out=np.full(len(ranges), np.inf),
                    where=np.abs(divisor) > 1e-6,
                )
                across = origin[1 - axis] + t * directions[:, 1 - axis]
                valid = (
                    valid_range & np.isfinite(across) & (t > 0) & (t <= ranges + 0.10)
                )
                for ray in np.flatnonzero(valid):
                    index = math.floor(across[ray] / self.size)
                    edge = ("v", line, index) if axis == 0 else ("h", index, line)
                    if edge not in self.required_edges:
                        continue
                    fraction = across[ray] / self.size - index
                    slot = math.floor((fraction - 0.1) / 0.05)
                    if not 0 <= slot < 16:
                        continue
                    if abs(t[ray] - ranges[ray]) <= 0.10:
                        self.hits[edge][slot] = True
                    elif t[ray] < ranges[ray] - 0.20:
                        self.free[edge][slot] = True

    def checked_edges(self):
        checked = set()
        for edge in self.required_edges:
            if self.truth[edge]:
                if self.hits[edge].mean() >= 0.65:
                    checked.add(edge)
            else:
                run = 0
                for free in self.free[edge]:
                    run = run + 1 if free else 0
                    if run >= 5:
                        checked.add(edge)
                        break
        return checked

    def write_result(self, final=False, error=None):
        checked = self.checked_edges()
        surveyed = {
            c for c in self.visited if all(e in checked for _, e in self.incident(c))
        }
        elapsed = self.state["elapsed"] if self.state else 0
        if error:
            status = "error"
        elif final:
            status = "complete"
        else:
            status = "running"
        result = dict(
            status=status,
            error=error,
            scenario=self.meta["name"],
            elapsed=elapsed,
            reachable_cells=len(self.reachable),
            visited_cells=len(self.visited),
            cell_coverage=len(self.visited) / len(self.reachable),
            surveyed_cells=len(surveyed),
            survey_coverage=len(surveyed) / len(self.reachable),
            checked_edges=len(checked),
            required_edges=len(self.required_edges),
            edge_coverage=len(checked) / len(self.required_edges),
            traversed_passages=len(self.traversed),
            first_full_cells=self.first_full_cells,
            first_full_survey=self.first_full_survey,
            success=self.first_full_survey is not None,
            algorithm_claimed_complete=self.claimed_complete,
            collisions=self.state["collisions"] if self.state else 0,
            truth_samples=self.samples,
            processed_scans=self.processed_scans,
            missing_cells=sorted(self.reachable - self.visited),
            unchecked_edges=sorted(self.required_edges - checked),
            station_cells_visited=[
                s["id"]
                for s in self.meta["stations"]
                if tuple(s["cell"]) in self.visited
            ],
        )
        path = self.folder / ("result.json" if final else "progress.json")
        staging = path.with_suffix(".pending")
        staging.write_text(json.dumps(result, ensure_ascii=False, indent=2))
        staging.replace(path)
        self.trajectory.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    rclpy.init()
    observer = Observer(args.folder)
    started = time.monotonic()
    try:
        while not observer.done and time.monotonic() - started < 1200:
            rclpy.spin_once(observer, timeout_sec=0.1)
        if not observer.done:
            observer.write_result(
                final=True, error="wall timeout or missing scene telemetry"
            )
    finally:
        observer.trajectory.close()
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
