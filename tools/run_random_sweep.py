#!/usr/bin/env python3
"""Sequential random-seed sweep through the organizer's run_eval.sh.

Run as a user with Docker access; DOCKER_CONFIG is inherited.
Usage: python3 tools/run_random_sweep.py OUTPUT_DIRECTORY
Create OUTPUT_DIRECTORY/STOP to stop after the current trial.
"""

import argparse
import csv
import itertools
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import time

import yaml


SCENE_IMAGE = "ghcr.io/ssskiv/virtual-courier-scene@sha256:12c3a340e42859108e67f62b12b2637908b2e89d1cd50a7259a42725b6c5d6d8"
SOLUTION_IMAGE = "ghcr.io/ssskiv/virtual-courier-solution@sha256:e06619c2863a3ff79c9a8a80153180afc055297325dfeea1acd47f1bcc66ae73"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    revision = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=repo, text=True
    ).strip()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    harness = out / "harness"
    (harness / "tools").mkdir(parents=True)
    for filename in ("docker-compose.yaml", "docker-compose.eval.yaml"):
        shutil.copyfile(repo / filename, harness / filename)
    shutil.copyfile(repo / "tools/run_eval.sh", harness / "tools/run_eval.sh")
    compose_path = harness / "docker-compose.yaml"
    compose = yaml.safe_load(compose_path.read_text())
    compose["services"]["scene"]["image"] = SCENE_IMAGE
    compose["services"]["solution"]["image"] = SOLUTION_IMAGE
    compose_path.write_text(yaml.safe_dump(compose, sort_keys=False))
    for directory in ("scenarios", "results", "code/src"):
        (out / directory).mkdir(parents=True)
    shutil.copytree(repo / "src/solution", out / "code/src/solution",
                    ignore=shutil.ignore_patterns("build", "install", "log", "__pycache__"))
    config_path = out / "code/src/solution/config/cell_explorer.yaml"
    config = yaml.safe_load(config_path.read_text())
    base = yaml.safe_load((repo / "scene/scenarios/public_1.yaml").read_text())
    sampler_seed = random.SystemRandom().randrange(2**64)
    rng = random.Random(sampler_seed)
    variants = list(itertools.product((0.5, 1.0), (1.5, 2.5, 3.5)))
    manifest = dict(revision=revision, sampler_seed=sampler_seed,
                    variants=variants, variants_per_seed=3,
                    scene_image=SCENE_IMAGE, solution_image=SOLUTION_IMAGE,
                    launcher="tools/run_eval.sh", mode="realtime")
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    env = dict(os.environ, ROS_DOMAIN_ID="93", WEBOTS_GUI="false",
               WEBOTS_MODE="realtime", SCENARIO_DIR=str(out / "scenarios"),
               RESULTS_DIR=str(out / "results"))
    index = 0
    seeds = set()
    while not (out / "STOP").exists():
        seed = rng.randrange(1, 1_000_000_000)
        if seed in seeds:
            continue
        seeds.add(seed)
        selected = rng.sample(variants, 3)
        with (out / "plan.jsonl").open("a") as stream:
            stream.write(json.dumps(dict(seed=seed, variants=selected)) + "\n")
        name = f"random_{seed}"
        scenario = dict(base, name=name, seed=seed, time_limit=600)
        (out / "scenarios" / f"{name}.yaml").write_text(
            yaml.safe_dump(scenario, sort_keys=False))
        for period, extension in selected:
            if (out / "STOP").exists():
                return
            index += 1
            params = config["cell_explorer"]["ros__parameters"]
            params.update(update_period=period, extension_distance=extension)
            config_path.write_text(yaml.safe_dump(config, sort_keys=False))
            team = f"sweep_{index:06d}"
            row = dict(index=index, seed=seed, revision=revision,
                       update_period=period, extension_distance=extension)
            print(f"Starting {row}", flush=True)
            started = time.monotonic()
            with (out / f"{team}.log").open("w") as log:
                try:
                    result = subprocess.run(
                        ["bash", "tools/run_eval.sh", team, str(out / "code"),
                         "-", name, "1"], cwd=harness, env=env, stdout=log,
                        stderr=subprocess.STDOUT, timeout=1800)
                    code = result.returncode
                except subprocess.TimeoutExpired:
                    subprocess.run(
                        ["docker", "compose", "-p", "vc_eval", "-f",
                         "docker-compose.yaml", "-f", "docker-compose.eval.yaml",
                         "down", "--remove-orphans"], cwd=harness, env=env,
                        stdout=log, stderr=subprocess.STDOUT, check=False)
                    code = 124
            matches = list((out / "results").glob(f"{team}_{name}_1_*.json"))
            judge = json.loads(matches[-1].read_text()) if matches else {}
            row.update(exit_code=code, finished=judge.get("finished", False),
                       score=judge.get("score"), elapsed=judge.get("elapsed"),
                       collisions=judge.get("collisions"),
                       delivered=sum(c.get("delivered", False)
                                     for c in judge.get("cargos", {}).values()),
                       wall_seconds=round(time.monotonic() - started, 2))
            with (out / "summary.jsonl").open("a") as stream:
                stream.write(json.dumps(row) + "\n")
            with (out / "summary.csv").open("a") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(row))
                if index == 1:
                    writer.writeheader()
                writer.writerow(row)
            shutil.rmtree(harness / "eval_work" / team)
            print(row, flush=True)


if __name__ == "__main__":
    main()
