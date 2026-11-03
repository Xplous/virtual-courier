#!/usr/bin/env python3
"""Run isolated headless scenes and save independent coverage measurements."""

import argparse
import csv
import json
import os
import queue
import shlex
import shutil
import signal
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
IMAGE = "ghcr.io/ssskiv/virtual-courier-scene:latest"


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


class Batch:
    def __init__(self, output, seeds, workers):
        self.output, self.seeds = output.resolve(), seeds
        self.output.mkdir(parents=True, exist_ok=False)
        (self.output / "COLCON_IGNORE").touch()
        self.slots = queue.Queue()
        for i in range(workers):
            self.slots.put(88 + i)
        self.lock = threading.Lock()
        self.folders = {}
        self.workers = workers

    def prepare(self):
        source = self.output / "snapshot_ws/src/solution"
        shutil.copytree(
            ROOT / "src/solution",
            source,
            ignore=shutil.ignore_patterns("build", "install", "log", "__pycache__"),
        )
        for script in ["run_exploration_batch.py", "observe_exploration.py"]:
            shutil.copy2(ROOT / "tools" / script, self.output / script)
        shutil.copy2(
            ROOT / "docker/scene/entrypoint.sh", self.output / "scene_entrypoint.sh"
        )
        (self.output / "scene_entrypoint.sh").chmod(0o755)
        (self.output / "fastdds.xml").write_text(
            (ROOT / "src/solution/config/fastdds.xml").read_text()
        )
        revision = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip()
        (self.output / "manifest.json").write_text(
            json.dumps(
                dict(
                    revision=revision,
                    seeds=self.seeds,
                    workers=self.workers,
                    mode="realtime",
                    gui=False,
                    simulation_limit=600,
                    solution_launch="solution start.launch.py gui:=false",
                    observer_uses_privileged_data=True,
                    solution_receives_privileged_data=False,
                    criteria="all reachable cell interiors visited and all incident edges observed by lidar",
                ),
                indent=2,
            )
        )
        with (self.output / "build.log").open("w") as log:
            subprocess.run(
                [
                    "bash",
                    "-c",
                    "source /opt/ros/humble/setup.bash\n"
                    "colcon build --symlink-install --executor sequential",
                ],
                cwd=self.output / "snapshot_ws",
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
            )
        base = yaml.safe_load((ROOT / "scene/scenarios/public_1.yaml").read_text())
        for seed in self.seeds:
            folder = self.output / f"seed_{seed}"
            (folder / "scenarios").mkdir(parents=True)
            (folder / "ros_logs").mkdir()
            self.folders[seed] = folder
            scenario = dict(base, name=f"batch_{seed}", seed=seed, time_limit=600)
            (folder / "scenarios/scenario.yaml").write_text(
                yaml.safe_dump(scenario, sort_keys=False)
            )
        self.summary()

    def summary(self):
        with self.lock:
            rows = []
            for seed in self.seeds:
                folder = self.folders[seed]
                path = folder / "result.json"
                if not path.exists():
                    path = folder / "progress.json"
                row = (
                    json.loads(path.read_text())
                    if path.exists()
                    else dict(status="queued")
                )
                rows.append(dict(seed=seed, **row))
            fields = [
                "seed",
                "status",
                "success",
                "elapsed",
                "visited_cells",
                "reachable_cells",
                "cell_coverage",
                "survey_coverage",
                "edge_coverage",
                "first_full_cells",
                "first_full_survey",
                "collisions",
                "algorithm_claimed_complete",
                "error",
            ]
            staging = self.output / "summary.json.pending"
            staging.write_text(json.dumps(rows, ensure_ascii=False, indent=2))
            staging.replace(self.output / "summary.json")
            staging = self.output / "summary.csv.pending"
            with staging.open("w") as out:
                writer = csv.DictWriter(out, fieldnames=fields, extrasaction="ignore")
                writer.writeheader()
                writer.writerows(rows)
            staging.replace(self.output / "summary.csv")

    def run_one(self, seed):
        domain = self.slots.get()
        folder = self.folders[seed]
        name = f"vc_coverage_{os.getpid()}_{seed}"
        nav = observer = scene_log = None
        files = []
        try:
            env = dict(
                os.environ,
                ROS_DOMAIN_ID=str(domain),
                RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                FASTRTPS_DEFAULT_PROFILES_FILE=str(self.output / "fastdds.xml"),
                ROS_LOG_DIR=str(folder / "ros_logs"),
                OPENBLAS_NUM_THREADS="1",
                OMP_NUM_THREADS="2",
            )
            command = [
                "docker",
                "run",
                "-d",
                "--init",
                "--name",
                name,
                "--network",
                "host",
                "--ipc",
                "private",
                "-e",
                f"ROS_DOMAIN_ID={domain}",
                "-e",
                "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
                "-e",
                "FASTRTPS_DEFAULT_PROFILES_FILE=/batch/fastdds.xml",
                "-e",
                "WEBOTS_GUI=false",
                "-e",
                "WEBOTS_MODE=realtime",
                "-e",
                "VC_EVAL=true",
                "-e",
                f"WEBOTS_DISPLAY={100+domain}",
                "-e",
                f"WEBOTS_PORT={12000+domain}",
                "-e",
                "SCENARIO=/scenarios/scenario.yaml",
                "-e",
                f"VC_TEAM=coverage_{seed}",
                "-v",
                f"{folder}/scenarios:/scenarios:ro",
                "-v",
                f"{folder}:/results",
                "-v",
                f"{self.output}/fastdds.xml:/batch/fastdds.xml:ro",
                "-v",
                f"{self.output}/scene_entrypoint.sh:/entrypoint.sh:ro",
            ]
            if Path("/dev/dri").exists():
                for device in Path("/dev/dri").iterdir():
                    if device.is_char_device():
                        command += ["--device", str(device)]
            command += [IMAGE]
            (folder / "runtime.json").write_text(
                json.dumps(
                    dict(domain=domain, container=name, command=command), indent=2
                )
            )
            subprocess.run(command, check=True, capture_output=True, text=True)
            log = (folder / "scene.log").open("w")
            files.append(log)
            scene_log = subprocess.Popen(
                ["docker", "logs", "-f", name],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                copied = subprocess.run(
                    [
                        "docker",
                        "cp",
                        name + ":/scene/webots/worlds/current.json",
                        str(folder / "world.json"),
                    ],
                    capture_output=True,
                )
                if copied.returncode == 0:
                    break
                time.sleep(1)
            else:
                raise RuntimeError("Scene did not generate metadata")
            log = (folder / "observer.log").open("w")
            files.append(log)
            observer = subprocess.Popen(
                ["python3", str(self.output / "observe_exploration.py"), str(folder)],
                cwd=ROOT,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            log = (folder / "navigation.log").open("w")
            files.append(log)
            launch = json.loads((self.output / "manifest.json").read_text())[
                "solution_launch"
            ]
            nav = subprocess.Popen(
                [
                    "bash",
                    "-c",
                    'source "$1"\n' 'shift\nexec ros2 launch "$@"',
                    "batch",
                    str(self.output / "snapshot_ws/install/setup.bash"),
                    *shlex.split(launch),
                ],
                cwd=ROOT,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            print(f"seed {seed}: started, ROS_DOMAIN_ID={domain}", flush=True)
            deadline = time.monotonic() + 1230
            while observer.poll() is None and time.monotonic() < deadline:
                self.summary()
                if nav.poll() is not None:
                    raise RuntimeError(f"Navigation launch exited: {nav.returncode}")
                scene = subprocess.run(
                    ["docker", "inspect", "--format", "{{.State.Running}}", name],
                    capture_output=True,
                    text=True,
                )
                if (
                    scene.stdout.strip() != "true"
                    and not (folder / "result.json").exists()
                ):
                    raise RuntimeError(
                        "Scene exited before evaluation finished; see scene.log"
                    )
                time.sleep(5)
            if not (folder / "result.json").exists():
                raise RuntimeError(
                    f"Observer did not save a result, exit={observer.poll()}"
                )
            result = json.loads((folder / "result.json").read_text())
            print(
                f'seed {seed}: {result["status"]}, cells={result["visited_cells"]}/{result["reachable_cells"]}, '
                f'survey={result["survey_coverage"]:.1%}, elapsed={result["elapsed"]}',
                flush=True,
            )
        except Exception as error:
            path = folder / "progress.json"
            result = json.loads(path.read_text()) if path.exists() else {}
            result.update(status="error", error=str(error), success=False)
            (folder / "result.json").write_text(json.dumps(result, indent=2))
            print(f"seed {seed}: ERROR {error}", flush=True)
        finally:
            stop_process(nav)
            stop_process(observer)
            subprocess.run(["docker", "rm", "-f", name], capture_output=True)
            stop_process(scene_log)
            for file in files:
                file.close()
            self.slots.put(domain)
            self.summary()

    def run(self):
        self.prepare()
        with ThreadPoolExecutor(max_workers=self.workers) as pool:
            list(pool.map(self.run_one, self.seeds))
        print("Batch complete: " + str(self.output / "summary.csv"), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seeds", type=int, nargs="+", required=True)
    parser.add_argument("--workers", type=int, default=2)
    args = parser.parse_args()
    Batch(args.output, args.seeds, args.workers).run()
