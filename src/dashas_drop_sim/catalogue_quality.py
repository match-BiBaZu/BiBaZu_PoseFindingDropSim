"""Publish geometric stability on existing grouped catalogues without physics."""
from __future__ import annotations
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tempfile

from .pose_quality import analyze_pose_quality, sha256, QualityCancelled

def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))

def write(path, data):
    Path(path).write_bytes((json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode())

def verify(folder):
    expected = read(folder / "files.sha256.json")
    actual = {p.relative_to(folder).as_posix() for p in folder.rglob("*") if p.is_file() and p.name != "files.sha256.json"}
    if actual != set(expected):
        raise ValueError(f"Catalogue inventory differs: {folder}")
    for name, digest in expected.items():
        file = (folder / name).resolve()
        if not file.is_relative_to(folder.resolve()) or sha256(file) != digest:
            raise ValueError(f"Catalogue checksum differs: {file}")

def annotate_catalogue(data, mesh_path, config_hash, *, evidence=None, progress=lambda message: None, check_cancel=lambda: None):
    """Return an additive catalogue and detached report with no checksum cycle."""
    original = deepcopy(data)
    settings = data["settings"]
    report = analyze_pose_quality(mesh_path, data["poses"], settings["recognition"],
        alpha_deg=settings["transverse_angle_deg"], beta_deg=settings["longitudinal_angle_deg"],
        evidence=evidence, progress=progress, check_cancel=check_cancel)
    report.update(workpiece=data["workpiece"], condition=data["condition"], config_sha256=config_hash)
    annotated = deepcopy(data)
    for row in annotated["poses"]:
        row["pose_quality"] = deepcopy(report["poses"][row["id"]])
    annotated["pose_quality_assessment"] = {k: deepcopy(v) for k, v in report.items() if k != "poses"}
    annotated["pose_quality_report"] = "pose_quality.json"
    assert original == data
    return annotated, report

def _readme(text, reports):
    text = text.split("\n## Geometric stability\n")[0]
    lines, condition, rows = [], None, []
    order = {"high": 0, "medium": 1, "low": 2, "unknown": 3}
    def flush():
        lines.extend(row for _, row in sorted(rows, key=lambda item: item[0]))
        rows.clear()
    for line in text.splitlines():
        if " Stability |" in line and line.startswith("| Pose |"):
            line = "|".join(line.split("|")[:8]) + "|"
        elif line.startswith("| P") and len(line.split("|")) > 9:
            line = "|".join(line.split("|")[:8]) + "|"
        elif line.startswith("| --- | --- | --- |") and len(line.split("|")) > 9:
            line = "|".join(line.split("|")[:8]) + "|"
        if line == "## Moving belt":
            flush(); condition = "moving"
        elif line == "## Stationary belt":
            flush(); condition = "stationary"
        report = reports.get(condition)
        if report and line.startswith("| Pose | Group |"):
            flush(); lines.append(line + " Stability | Lift to tip (mm) | Easiest target |")
        elif report and line.startswith("| --- | --- | --- |"):
            lines.append(line + " --- | ---: | --- |")
        elif report and line.startswith("| P") and (match := re.search(r"<code>([^<]+)</code>", line)):
            entry = report["poses"][match[1]]
            lift = entry["escape_lift_mm"]
            target = entry["escape_target_source_id"] or "unknown"
            if entry["escape_within_group"]:
                target += " (rocking within group)"
            value = f"{lift:.4f}" if lift is not None else "unknown"
            rows.append((order[entry["stability"]], line + f" {entry['stability'].title()} | {value} | {target} |"))
        else:
            flush(); lines.append(line)
    flush()
    lines += ["", "## Geometric stability", "",
        "Stability is shown High, Medium, then Low. High needs at least 0.5 mm of sampled centre-of-mass lift to tip; "
        "Medium needs 0.1 to below 0.5 mm; Low needs below 0.1 mm. Unknown means no distinct destination could be assessed. "
        "Frequency, pose ranks and groups are unchanged. Within-group rocking does not classify the whole group as unstable. "
        "These geometry-only estimates are not measured transport survival probabilities.", ""]
    for name in reports:
        lines.append(f"[{name.title()} stability report](catalogue/grouped_10000/{name}/pose_quality.json)")
    return "\n".join(lines) + "\n"

def annotate_repository(repo, *, condition="both", progress=lambda message: None, check_cancel=lambda: None):
    repo = Path(repo).resolve()
    index = read(repo / "index.json")
    conditions = ("moving", "stationary") if condition == "both" else (condition,)
    if any(c not in {"moving", "stationary"} for c in conditions):
        raise ValueError("Invalid belt condition.")
    if len(index["workpieces"]) != 39 or len({p["workpiece"] for p in index["workpieces"]}) != 39:
        raise ValueError("Expected the published 39-workpiece catalogue index.")
    # All geometry and publication payloads are staged before any replacement.
    with tempfile.TemporaryDirectory(prefix=".pose-quality-", dir=repo / ".git") as temporary:
        staging = Path(temporary)
        replacements, texts = [], {}
        source_hashes = {repo / name: sha256(repo / name) for name in ("index.json", "README.md")}
        for item in index["workpieces"]:
            part = item["workpiece"]
            source_hashes[repo / part / "README.md"] = sha256(repo / part / "README.md")
            reports = {}
            for belt in conditions:
                check_cancel()
                record = item["grouped_10000_catalogues"][belt]
                file = (repo / record["path"]).resolve()
                if not file.is_relative_to(repo / part) or file.name != "catalogue.json":
                    raise ValueError("Unsafe catalogue path.")
                folder = file.parent
                verify(folder)
                data = read(file)
                if data["workpiece"] != part or data["condition"] != belt:
                    raise ValueError("Catalogue identity differs from index.")
                mesh = (repo / data["source"]["mesh_path"]).resolve()
                config = (repo / data["source"]["run_dir"] / "config.json").resolve()
                if not mesh.is_relative_to(repo / part) or not config.is_relative_to(repo / part):
                    raise ValueError("Unsafe mesh or configuration path.")
                if sha256(mesh) != data["source"]["mesh_sha256"] or sha256(config) != data["source"]["config_sha256"]:
                    raise ValueError("Mesh or configuration checksum differs.")
                evidence_file = folder / data["grouping_evidence"]
                if not evidence_file.resolve().is_relative_to(folder) or sha256(evidence_file) != data["grouping_evidence_sha256"]:
                    raise ValueError("Grouping evidence checksum differs.")
                source_hashes.update({file: sha256(file), mesh: sha256(mesh), config: sha256(config)})
                progress(f"{part} {belt}: assessing geometric stability")
                annotated, report = annotate_catalogue(data, mesh, sha256(config), evidence=read(evidence_file),
                    progress=progress, check_cancel=check_cancel)
                staged = staging / part / belt
                shutil.copytree(folder, staged)
                write(staged / "catalogue.json", annotated)
                report.update(source_sha256=sha256(staged / "catalogue.json"),
                              observations_sha256=sha256(staged / "catalogue.json"))
                write(staged / "pose_quality.json", report)
                write(staged / "files.sha256.json", {p.relative_to(staged).as_posix(): sha256(p)
                    for p in sorted(staged.rglob("*")) if p.is_file() and p.name != "files.sha256.json"})
                verify(staged)
                reports[belt] = report
                record.update(stability_counts=report["stability_counts"], pose_quality_report=(folder / "pose_quality.json").relative_to(repo).as_posix())
                replacements.append((folder, staged))
            part_readme = repo / part / "README.md"
            texts[part_readme] = _readme(part_readme.read_text(encoding="utf-8"), reports).encode()
        root_readme = repo / "README.md"
        texts[root_readme] = (root_readme.read_text(encoding="utf-8").split("\n## Geometric stability\n")[0] + "\n## Geometric stability\n\n"
            "Every current moving and stationary catalogue includes pose stability computed by DropSim after grouping. "
            "High: lift ≥ 0.5 mm; Medium: 0.1–<0.5 mm; Low: <0.1 mm. Unknown is reported when no distinct destination is assessable. "
            "See the Stability column and detached stability reports for each workpiece. Original poses, incidence and grouping are unchanged.\n").encode()
        texts[repo / "index.json"] = (json.dumps(index, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode()
        check_cancel()
        for file, digest in source_hashes.items():
            if sha256(file) != digest:
                raise ValueError("An input changed during assessment; no catalogues were replaced.")
        for folder, _ in replacements:
            verify(folder)
        backups, previous = [], {file: file.read_bytes() for file in texts}
        try:
            for number, (folder, staged) in enumerate(replacements):
                backup = staging / f"backup-{number}"
                os.replace(folder, backup)
                backups.append((folder, backup))
                os.replace(staged, folder)
            for file, content in texts.items():
                file.write_bytes(content)
        except BaseException:
            for folder, backup in reversed(backups):
                if folder.exists():
                    shutil.rmtree(folder)
                os.replace(backup, folder)
            for file, content in previous.items():
                file.write_bytes(content)
            raise
    return {"catalogues": len(replacements), "condition": condition, "stability": "published"}

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--condition", choices=("both", "moving", "stationary"), default="both")
    parser.add_argument("--cancel-file", type=Path)
    args = parser.parse_args(argv)
    def cancel():
        if args.cancel_file and args.cancel_file.exists():
            raise QualityCancelled("Stability update cancelled before publication.")
    try:
        result = annotate_repository(args.repo, condition=args.condition,
            progress=lambda message: print(json.dumps({"message": message}), flush=True), check_cancel=cancel)
        print(json.dumps({"status": "completed", **result}), flush=True)
        return 0
    except QualityCancelled as error:
        print(json.dumps({"status": "cancelled", "message": str(error)}), flush=True)
        return 130

if __name__ == "__main__":
    raise SystemExit(main())
