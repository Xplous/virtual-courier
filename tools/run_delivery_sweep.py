#!/usr/bin/env python3
"""Последовательные заезды доставки с перебором параметров из YAML."""

import argparse
import csv
import itertools
import json
import shutil
import subprocess
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def docker(*args, check=True):
    return subprocess.run(
        ["docker", *map(str, args)], check=check, capture_output=True, text=True
    )


def write_json(path, value):
    temporary = path.with_suffix(".pending")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2))
    temporary.replace(path)


def cpu_times():
    values = list(map(int, Path("/proc/stat").read_text().splitlines()[0].split()[1:]))
    return sum(values[:8]), values[3] + values[4]


def memory_mb():
    values = {
        line.split(":")[0]: int(line.split()[1])
        for line in Path("/proc/meminfo").read_text().splitlines()
    }
    return (values["MemTotal"] - values["MemAvailable"]) / 1024


def make_plan(config):
    defaults = yaml.safe_load(
        (ROOT / "src/solution/config/cell_explorer.yaml").read_text()
    )["cell_explorer"]["ros__parameters"]
    matrix = config["matrix"]
    for key, values in matrix.items():
        if key not in defaults or not isinstance(values, list) or not values:
            raise ValueError(f"Недопустимый параметр или набор значений: {key}")
    plan = []
    for combination in itertools.product(*matrix.values()):
        parameters = dict(zip(matrix, combination))
        for key, value in parameters.items():
            if isinstance(defaults[key], float):
                parameters[key] = float(value)
        for seed in config["seeds"]:
            plan.append(
                dict(index=len(plan) + 1, seed=seed, parameters=parameters.copy())
            )
    if not plan:
        raise ValueError("Пустой план заездов")
    return plan


def save_summary(output, rows, parameter_names):
    write_json(output / "summary.json", rows)
    fields = [
        "index",
        "seed",
        *parameter_names,
        "status",
        "score",
        "delivered",
        "success",
        "elapsed",
        "collisions",
        "wall_seconds",
        "cpu_mean_pct",
        "cpu_max_pct",
        "ram_peak_mb",
        "error",
    ]
    with (output / "summary.csv").open("w") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def run_one(output, config, trial):
    folder = output / f'{trial["index"]:03d}_seed_{trial["seed"]}'
    folder.mkdir()
    cores = folder / "cores"
    cores.mkdir()
    dumped = len(list(output.glob("*/cores/core.*")))
    core_limit = (
        config["core_limit_mb"] * 1024 * 1024
        if dumped < config["max_core_dumps"]
        else 0
    )
    solution = folder / "solution"
    shutil.copytree(output / "source_solution", solution)
    params_path = solution / "config/cell_explorer.yaml"
    params = yaml.safe_load(params_path.read_text())
    params["cell_explorer"]["ros__parameters"].update(trial["parameters"])
    params_path.write_text(yaml.safe_dump(params, sort_keys=False))
    scenario = yaml.safe_load((output / "base_scenario.yaml").read_text())
    scenario.update(seed=trial["seed"], time_limit=config["simulation_limit"])
    (folder / "scenario.yaml").write_text(yaml.safe_dump(scenario, sort_keys=False))
    shutil.copy2(output / "observe_delivery.py", folder / "observe_delivery.py")
    write_json(folder / "trial.json", dict(trial, core_limit_bytes=core_limit))
    names = [
        f'vc_sweep_{output.name}_{trial["index"]}_{role}'
        for role in ("scene", "solution")
    ]
    domain = config["ros_domain_id"]
    common = [
        "--init",
        "--network",
        "host",
        "--ipc",
        "private",
        "-e",
        f"ROS_DOMAIN_ID={domain}",
        "-e",
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
        "-e",
        "FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastdds.xml",
        "-v",
        f"{output}/fastdds.xml:/config/fastdds.xml:ro",
    ]
    started = time.monotonic()
    previous_cpu = cpu_times()
    cpu_samples, ram_samples = [], []
    result = dict(
        index=trial["index"],
        seed=trial["seed"],
        **trial["parameters"],
        status="error",
        error="",
    )
    interrupted = False
    try:
        docker(
            "run",
            "-d",
            "--name",
            names[0],
            *common,
            "--shm-size",
            "256m",
            "--ulimit",
            f"core={core_limit}:{core_limit}",
            "-v",
            f"{cores}:/cores",
            "-e",
            "WEBOTS_GUI=false",
            "-e",
            "WEBOTS_MODE=realtime",
            "-e",
            f"WEBOTS_DISPLAY={100+domain}",
            "-e",
            f"WEBOTS_PORT={12000+domain}",
            "-e",
            "VC_EVAL=true",
            "-e",
            "VC_TEAM=sweep",
            "-e",
            "SCENARIO=/scenario.yaml",
            "-e",
            "LIBGL_ALWAYS_SOFTWARE=1",
            "-e",
            "GALLIUM_DRIVER=llvmpipe",
            "-e",
            "LP_NUM_THREADS=1",
            "-v",
            f"{output}/source_scene:/scene",
            "-v",
            f"{folder}/scenario.yaml:/scenario.yaml:ro",
            "-v",
            f"{folder}:/results",
            "-v",
            f"{output}/scene_entrypoint.sh:/entrypoint.sh:ro",
            "--entrypoint",
            "bash",
            config["scene_image"],
            "/entrypoint.sh",
        )
        docker(
            "run",
            "-d",
            "--name",
            names[1],
            *common,
            "-e",
            "OPENBLAS_NUM_THREADS=1",
            "-e",
            "OMP_NUM_THREADS=2",
            "-e",
            "ROS_LOG_DIR=/bench/ros_logs",
            "-v",
            f"{solution}:/ros_ws/src/solution",
            "-v",
            f"{folder}:/bench",
            "--entrypoint",
            "bash",
            config["solution_image"],
            "-lc",
            "set -e; source /opt/ros/humble/setup.bash; cd /ros_ws; "
            "colcon build --symlink-install; source install/setup.bash; "
            "exec ros2 launch solution start.launch.py gui:=false",
        )
        docker(
            "exec",
            "-d",
            names[1],
            "bash",
            "-lc",
            "source /opt/ros/humble/setup.bash; "
            "exec python3 /bench/observe_delivery.py > /bench/observer.log 2>&1",
        )
        with (folder / "resources.csv").open("w", buffering=1) as file:
            writer = csv.writer(file)
            writer.writerow(["wall_seconds", "cpu_pct", "ram_used_mb"])
            while True:
                time.sleep(5)
                current_cpu = cpu_times()
                total = current_cpu[0] - previous_cpu[0]
                busy = 100 * (1 - (current_cpu[1] - previous_cpu[1]) / max(total, 1))
                previous_cpu = current_cpu
                ram = memory_mb()
                cpu_samples.append(busy)
                ram_samples.append(ram)
                wall = time.monotonic() - started
                writer.writerow([round(wall, 2), round(busy, 2), round(ram, 2)])
                states = [
                    json.loads(line)
                    for line in docker(
                        "inspect", "--format", "{{json .State}}", *names
                    ).stdout.splitlines()
                ]
                if not states[0]["Running"]:
                    result["status"] = "scene_error"
                    result["error"] = (
                        f'Webots exit={states[0]["ExitCode"]}, '
                        f'OOMKilled={states[0]["OOMKilled"]}'
                    )
                    break
                if not states[1]["Running"]:
                    result["status"] = "solution_exited"
                    result["error"] = f'Solution exit={states[1]["ExitCode"]}'
                    break
                if wall >= config["wall_timeout"]:
                    result["status"] = "wall_timeout"
                    break
    except KeyboardInterrupt:
        result["status"] = "interrupted"
        interrupted = True
    except subprocess.CalledProcessError as error:
        result["error"] = error.stderr.strip()
    finally:
        docker("stop", "-t", "5", *names, check=False)
        for name, role in zip(names, ("scene", "solution")):
            logs = docker("logs", name, check=False)
            (folder / f"{role}.log").write_text(logs.stdout + logs.stderr)
        containers = docker("inspect", *names, check=False)
        (folder / "containers.json").write_text(containers.stdout)
        docker("rm", *names, check=False)
    judge_files = list(folder.glob("sweep_*.json"))
    judge = json.loads(judge_files[0].read_text()) if judge_files else {}
    if judge.get("finished"):
        result["status"] = "complete"
        result["error"] = ""
    cargos = judge.get("cargos", {})
    result.update(
        score=judge.get("score"),
        elapsed=judge.get("elapsed"),
        collisions=judge.get("collisions"),
        delivered=sum(cargo["delivered"] for cargo in cargos.values()),
        success=judge.get("finish_reason") == "все грузы доставлены",
        wall_seconds=round(time.monotonic() - started, 2),
        cpu_mean_pct=(
            round(sum(cpu_samples) / len(cpu_samples), 2) if cpu_samples else None
        ),
        cpu_max_pct=round(max(cpu_samples), 2) if cpu_samples else None,
        ram_peak_mb=round(max(ram_samples), 2) if ram_samples else None,
    )
    write_json(folder / "result.json", result)
    return result, interrupted


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=ROOT / "tools/delivery_sweep.yaml"
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    plan = make_plan(config)
    if args.dry_run:
        print(json.dumps(plan, indent=2))
        print(f"Всего заездов: {len(plan)}")
        return
    if args.output is None:
        parser.error("--output обязателен для запуска")
    if (
        config["max_core_dumps"]
        and Path("/proc/sys/kernel/core_pattern").read_text().strip()
        != "/cores/core.%e.%p.%t"
    ):
        parser.error(
            "Для дампов настройте kernel.core_pattern=/cores/core.%e.%p.%t или задайте max_core_dumps: 0"
        )
    for image in (config["scene_image"], config["solution_image"]):
        docker("image", "inspect", image)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "COLCON_IGNORE").touch()
    write_json(output / "plan.json", plan)
    write_json(
        output / "manifest.json",
        dict(
            revision=subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
            ).strip(),
            config=config,
            images=json.loads(
                docker(
                    "image", "inspect", config["scene_image"], config["solution_image"]
                ).stdout
            ),
        ),
    )
    shutil.copy2(__file__, output / "run_delivery_sweep.py")
    shutil.copy2(ROOT / "tools/observe_delivery.py", output / "observe_delivery.py")
    shutil.copy2(ROOT / config["scenario"], output / "base_scenario.yaml")
    shutil.copy2(ROOT / "docker/scene/entrypoint.sh", output / "scene_entrypoint.sh")
    shutil.copy2(ROOT / "src/solution/config/fastdds.xml", output / "fastdds.xml")
    shutil.copytree(
        ROOT / "src/solution",
        output / "source_solution",
        ignore=shutil.ignore_patterns(
            "build", "install", "log", "__pycache__", "camera_view.jpg"
        ),
    )
    shutil.copytree(
        ROOT / "scene",
        output / "source_scene",
        ignore=shutil.ignore_patterns("*.wbt", "*.json", ".*.wbproj", "__pycache__"),
    )
    rows = []
    for trial in plan:
        print(
            f'[{trial["index"]}/{len(plan)}] seed={trial["seed"]} {trial["parameters"]}',
            flush=True,
        )
        result, interrupted = run_one(output, config, trial)
        rows.append(result)
        save_summary(output, rows, list(config["matrix"]))
        print(
            f'  {result["status"]}: score={result["score"]}, delivered={result["delivered"]}',
            flush=True,
        )
        if interrupted:
            break
    print(f'Результаты: {output / "summary.csv"}', flush=True)


if __name__ == "__main__":
    main()
