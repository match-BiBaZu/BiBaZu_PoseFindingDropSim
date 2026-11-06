"""Run and verify the paired 100-drop conveyor validation for the bundled catalog."""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from datetime import datetime, timezone
import csv
import json
from pathlib import Path
import shutil
from statistics import median
import subprocess
import sys
from threading import Lock

import numpy as np
from scipy.spatial.transform import Rotation

from dashas_drop_sim.catalog import catalog_models, SOURCE_COMMIT
from dashas_drop_sim.cli import _config
from dashas_drop_sim.config import RunConfig
from dashas_drop_sim.runner import SETTLED_STATUSES, _sha256, _write_csv, _write_json


ROOT = Path(__file__).resolve().parents[1]
MODES = ("running", "stopped")
TRIALS = 100
FILES = ("config.json", "manifest.json", "trials.csv", "trials.jsonl", "summary.json",
         "frequencies.csv", "roughness_events.csv", "roughness_events.jsonl")
RELEASE = ("initial_quat_xyzw", "initial_pos_chute_mm", "actual_drop_height_mm", "actual_lateral_mm")


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def verified_trials(folder: Path, config: RunConfig, commit: str) -> list[dict]:
    if not all((folder / name).is_file() for name in FILES):
        raise ValueError("Missing run files")
    if read_json(folder / "config.json") != json.loads(json.dumps(config.to_dict())):
        raise ValueError("Run configuration differs from the validation profile")
    manifest = read_json(folder / "manifest.json")
    running = config.belt_mode == "running"
    if (manifest["software_commit"] != commit
            or manifest["mesh_source_sha256"] != _sha256(config.mesh_path)
            or manifest["belt_mode"] != config.belt_mode
            or manifest["group_poses"] is not False
            or manifest["effective_belt_speed_mm_s"] != (100 if running else 0)
            or manifest["belt_speed_variation"]["speed_range_mm_s"] != ([97, 103] if running else [0, 0])
            or manifest["belt_speed_variation"]["enabled"] is not running
            or manifest["belt_stop_observation"]["enabled"]):
        raise ValueError("Manifest does not match the requested condition")
    rows = [json.loads(line) for line in (folder / "trials.jsonl").read_text(encoding="utf-8").splitlines()]
    expected_seeds = [int(child.generate_state(1, dtype=np.uint32)[0])
                      for child in np.random.SeedSequence(config.seed).spawn(TRIALS)]
    if ([row["trial"] for row in rows] != list(range(TRIALS))
            or [row["seed"] for row in rows] != expected_seeds or len(set(expected_seeds)) != TRIALS):
        raise ValueError("Trial IDs or seeds are incomplete, duplicated or out of order")
    expected_settled = "settled" if running else "settled_stationary"
    settled = [row for row in rows if row["status"] in SETTLED_STATUSES]
    if any(row["status"] != expected_settled or row["pose_key"] != f"trial_{row['trial']:03d}"
           for row in settled):
        raise ValueError("A settled trial is grouped or has the wrong belt status")
    if any(bool(row["belt_speed_control_points"]) != running or row["stop_observation"] is not None for row in rows):
        raise ValueError("Trial belt speed or exit braking does not match the requested condition")
    with (folder / "trials.csv").open(encoding="utf-8", newline="") as stream:
        csv_rows = list(csv.DictReader(stream))
    keys = ("trial", "seed", "status", "pose_key")
    if ([tuple(row[key] for key in keys) for row in csv_rows]
            != [tuple(str(row[key]) for key in keys) for row in rows]):
        raise ValueError("Raw CSV and JSONL trial records disagree")
    summary = read_json(folder / "summary.json")
    if (summary["cancelled"] or summary["requested_trials"] != TRIALS
            or summary["completed_trials"] != TRIALS or summary["compute_stability"]
            or summary["disturbance_trials"] != 0 or summary["settled_trials"] != len(settled)
            or summary["status_counts"] != Counter(row["status"] for row in rows)
            or sum(row["count"] for row in summary["pose_frequencies"]) != TRIALS):
        raise ValueError("Run summary and trials do not reconcile")
    return rows


def run_condition(config: RunConfig, target: Path, commit: str) -> list[dict]:
    try:
        return verified_trials(target, config, commit)
    except (OSError, ValueError, KeyError):
        pass
    config.output_dir.mkdir(parents=True, exist_ok=True)
    for native in sorted(config.output_dir.glob("run_*"), reverse=True):
        try:
            rows = verified_trials(native, config, commit)
        except (OSError, ValueError, KeyError):
            continue
        break
    else:
        cfg_path = config.output_dir / f"{config.belt_mode}.json"
        _write_json(cfg_path, config.to_dict())
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        log = config.output_dir / f"{config.belt_mode}-{timestamp}.log"
        with log.open("w", encoding="utf-8") as stream:
            result = subprocess.run([sys.executable, "-m", "dashas_drop_sim", "run", "--config", str(cfg_path)],
                                    cwd=ROOT, stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError(f"Simulator exited with code {result.returncode}. See {log}")
        for line in reversed(log.read_text(encoding="utf-8", errors="replace").splitlines()):
            if line.startswith('{"event": "done"'):
                native = Path(json.loads(line)["run_dir"])
                break
        else:
            raise ValueError(f"No completion event in {log}")
        rows = verified_trials(native, config, commit)
    target.mkdir(parents=True, exist_ok=True)
    for name in FILES:
        shutil.copyfile(native / name, target / name)
    return verified_trials(target, config, commit)


def write_results(output: Path, results: dict, commit: str) -> bool:
    workpieces = []
    paired = []
    totals = {mode: Counter() for mode in MODES}
    for name, runs in results.items():
        angles = []
        pairs = []
        if all(mode in runs for mode in MODES):
            for running, stopped in zip(runs["running"], runs["stopped"], strict=True):
                if any(running[key] != stopped[key] for key in ("trial", "seed", *RELEASE)):
                    raise ValueError(f"Mismatched paired release for {name} trial {running['trial']}")
                row = {"workpiece": name, "trial": running["trial"], "seed": running["seed"]}
                for mode, record in zip(MODES, (running, stopped), strict=True):
                    row[f"{mode}_status"] = record["status"]
                    row[f"{mode}_final_quat_xyzw"] = record["final_quat_xyzw"]
                    row[f"{mode}_pose_id"] = record["pose_key"] if record["status"] in SETTLED_STATUSES else None
                if all(record["status"] in SETTLED_STATUSES for record in (running, stopped)):
                    first = Rotation.from_quat(running["final_quat_xyzw"])
                    second = Rotation.from_quat(stopped["final_quat_xyzw"])
                    angle = float(np.rad2deg((first.inv() * second).magnitude()))
                    row["angular_difference_deg"] = angle
                    angles.append(angle)
                pairs.append(row)
        paired.extend(pairs)
        for mode, rows in runs.items():
            counts = Counter(row["status"] for row in rows)
            totals[mode].update(counts)
            settled = sum(counts[status] for status in SETTLED_STATUSES)
            workpieces.append({"workpiece": name, "belt_mode": mode, "trials": len(rows),
                               "settled_trials": settled, "settlement_rate": settled / len(rows),
                               **{f"status_{status}": count for status, count in counts.items()},
                               "paired_trials": len(pairs), "both_settled_pairs": len(angles),
                               "median_paired_angle_deg": median(angles) if angles else None})
    status_fields = sorted({key for row in workpieces for key in row if key.startswith("status_")})
    for row in workpieces:
        for key in status_fields:
            row.setdefault(key, 0)
    _write_csv(output / "workpiece_summary.csv",
               ["workpiece", "belt_mode", "trials", "settled_trials", "settlement_rate", *status_fields,
                "paired_trials", "both_settled_pairs", "median_paired_angle_deg"], workpieces)
    _write_csv(output / "paired_trials.csv",
               ["workpiece", "trial", "seed", "running_status", "stopped_status",
                "running_pose_id", "stopped_pose_id", "running_final_quat_xyzw", "stopped_final_quat_xyzw",
                "angular_difference_deg"], paired)
    drops = sum(sum(counts.values()) for counts in totals.values())
    complete = drops == 7800 and len(workpieces) == 78 and len(paired) == 3900
    report = [
        "# Conveyor validation", "",
        f"Verified {drops:,} of 7,800 drops and {len(paired):,} of 3,900 paired trials.",
        f"Simulator commit `{commit}`. Bundled catalog source `{SOURCE_COMMIT}`.", "",
        "| Belt condition | Trials | Settled | Settlement rate | Other statuses |",
        "| --- | ---: | ---: | ---: | --- |",
    ]
    for mode, counts in totals.items():
        n = sum(counts.values())
        settled = sum(counts[status] for status in SETTLED_STATUSES)
        other = ", ".join(f"{status} {count}" for status, count in sorted(counts.items())
                          if status not in SETTLED_STATUSES)
        rate = f"{settled / n:.1%}" if n else "—"
        report.append(f"| {mode} | {n:,} | {settled:,} | {rate} | {other or '—'} |")
    report += [
        "", "Running trials were assessed at the 500 mm virtual exit with a nominal belt speed of 100 mm/s "
        "and ±3 mm/s variation. The settlement window checks support, orientation, and transverse motion.",
        "", "Stopped trials were assessed after 10 seconds at zero belt speed. The final half-second "
        "window checks support and orientation plus XYZ position range up to 1 mm and filtered speed up to "
        "1 mm/s. The two rates describe different measurement conditions.",
        "", "Trials used seed 42, four workers, and the bundled GUI contact and release settings. "
        "The batch ran with grouping, exit braking, disturbance sweeps, and catalog publishing disabled.",
        "", "Paired angles are the shortest rotation between the raw final orientations in degrees. "
        "They are reported for pairs in which both trials settled, with no workpiece symmetry reduction.",
        "", "The results describe the simulation under these settings. The shipped contact parameters "
        "are uncalibrated assumptions.", "",
    ]
    temporary = output / "report.md.tmp"
    temporary.write_text("\n".join(report), encoding="utf-8")
    temporary.replace(output / "report.md")
    return complete


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scratch", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=4, help="Concurrent workpieces, each using four workers")
    args = parser.parse_args()
    output, scratch = args.output.resolve(), args.scratch.resolve()
    output.mkdir(parents=True, exist_ok=True)
    base = _config(ROOT / "configs" / "gui-current.json")
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    models = catalog_models()
    if len(models) != 39 or not 1 <= args.jobs <= 4:
        parser.error("Expected 39 bundled workpieces and one to four concurrent jobs")
    index_file = output / "run_index.json"
    index = {name: {mode: {"status": "pending"} for mode in MODES} for name in models}
    index_lock = Lock()

    def update(name, mode, **record):
        with index_lock:
            index[name][mode] = record
            _write_json(index_file, {"software_commit": commit, "workpieces": index})
            if record["status"] != "running":
                print(f"{name} {mode} {record['status']}", flush=True)

    def run_workpiece(item):
        name, mesh = item
        runs = {}
        for mode in MODES:
            config = replace(base, mesh_path=mesh, output_dir=scratch / name, belt_mode=mode, group_poses=False,
                             trials=TRIALS, workers=4, seed=42, belt_stop_enabled=False, compute_stability=False,
                             catalog_repo=None, catalog_cad_dir=None, catalog_push=False)
            target = output / "runs" / name / mode
            update(name, mode, status="running")
            try:
                runs[mode] = run_condition(config, target, commit)
            except (OSError, ValueError, KeyError, RuntimeError) as error:
                update(name, mode, status="failed", error=str(error))
            else:
                update(name, mode, status="complete", path=target.relative_to(output).as_posix())
        return name, runs

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        results = dict(pool.map(run_workpiece, models.items()))
    return 0 if write_results(output, results, commit) else 1


if __name__ == "__main__":
    sys.exit(main())
