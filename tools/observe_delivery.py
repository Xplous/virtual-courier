#!/usr/bin/env python3
"""Сохраняет состояние судьи; запускается отдельно от алгоритма внутри ROS-контейнера."""

import json
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def main():
    folder = Path("/bench")
    rclpy.init()
    node = rclpy.create_node("delivery_observer")
    started = time.monotonic()
    clock = 0.0
    last_clock, last_wall = 0.0, started
    trace = (folder / "referee.jsonl").open("w", buffering=1)

    def on_clock(msg):
        nonlocal clock
        clock = msg.clock.sec + msg.clock.nanosec / 1e9

    def on_state(msg):
        nonlocal last_clock, last_wall
        now = time.monotonic()
        state = json.loads(msg.data)
        trace.write(
            json.dumps(dict(wall_seconds=now - started, **state), ensure_ascii=False)
            + "\n"
        )
        (folder / "referee_latest.json").write_text(
            json.dumps(state, ensure_ascii=False, indent=2)
        )
        if now - last_wall >= 5:
            progress = dict(
                wall_seconds=now - started,
                sim_time=clock,
                realtime_factor=(clock - last_clock) / (now - last_wall),
                score=state["score"],
                elapsed=state["elapsed"],
            )
            temporary = folder / "progress.pending"
            temporary.write_text(json.dumps(progress, indent=2))
            temporary.replace(folder / "progress.json")
            last_clock, last_wall = clock, now

    subscriptions = [
        node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data),
        node.create_subscription(String, "/referee/state", on_state, 10),
    ]
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        trace.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
