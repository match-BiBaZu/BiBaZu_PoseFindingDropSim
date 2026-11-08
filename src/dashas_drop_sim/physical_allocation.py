"""Allocate physical orientation labels to fixed, previously assessed pose groups.

This is annotation postprocessing. It never runs drops, assesses disturbances,
changes incidence, adds group members or selects new representatives.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path

from .grouping import analyze_groups
from .observed import ObservedPoseResolver

MODEL = "fixed-group-physical-orientations-v1"


def allocation_inputs(run, catalogue, mesh_path):
    """Content identity used by the exporter and the read-only plotter."""
    run = Path(run)
    files = {name: run / name for name in
             ("groups.json", "group_members.json", "manifest.json", "config.json")}
    files.update(physical_catalogue=Path(catalogue), mesh=Path(mesh_path))
    return {name: hashlib.sha256(path.read_bytes()).hexdigest()
            for name, path in files.items()}


def allocate_physical_poses(mesh_path, members, groups, recognition, references, *,
                            alpha_deg=45, beta_deg=0, barrier_mm=.5,
                            max_angle_deg=60, step_deg=.1):
    """Query existing groups by measured recognition, then seated basin evidence.

    Each nonmatching reference is probed independently, so references cannot
    bridge one another or grow the saved groups. A probe is accepted only if
    the published procedure leaves the complete original partition unchanged.
    Cross-group recognition ambiguity is preserved rather than guessed away.
    """
    owner = {}
    partitions = set()
    group_ids = set()
    for group in groups:
        group_id = group["group_id"]
        if group_id in group_ids or not group["members"]:
            raise ValueError("Expected unique nonempty saved groups")
        group_ids.add(group_id)
        partitions.add(frozenset(group["members"]))
        for member in group["members"]:
            if member in owner:
                raise ValueError("Source member belongs to multiple saved groups")
            owner[member] = group_id
    if len({m["id"] for m in members}) != len(members) or set(owner) != {m["id"] for m in members}:
        raise ValueError("Saved groups must partition all source members")
    if len({r["physical_pose_label"] for r in references}) != len(references):
        raise ValueError("Physical labels must be unique")
    resolver = ObservedPoseResolver(Path(mesh_path), data={"poses": members, "recognition": recognition})
    grouped = resolver.grouped({key: str(value) for key, value in owner.items()})
    evidence = []
    for reference in sorted(references, key=lambda p: p["physical_pose_label"]):
        quat = reference["quaternion_xyzw"]
        match = grouped.resolve(quat)
        row = dict(physical_pose_label=reference["physical_pose_label"],
                   quaternion_xyzw=deepcopy(quat), group_id=None,
                   status="unassigned", method=None, recognition_distance_deg=match.distance_deg,
                   candidate_group_ids=[], paths=[])
        if match.status == "matched":
            row.update(status="assigned", method="observed_orientation", group_id=int(match.pose_id))
        elif match.status == "ambiguous":
            row.update(status="ambiguous", reason="cross_group_orientation_ambiguity")
        else:
            probe_id = "physical_reference_query"
            while probe_id in owner:
                probe_id += "_"
            probe = dict(id=probe_id, quaternion_xyzw=quat)
            result = analyze_groups(mesh_path, [*members, probe], recognition,
                                    alpha_deg=alpha_deg, beta_deg=beta_deg,
                                    barrier_mm=barrier_mm, max_angle_deg=max_angle_deg, step_deg=step_deg)
            containing = next(set(g) for g in result["groups"] if probe_id in g)
            candidates = containing - {probe_id}
            row["candidate_group_ids"] = sorted({owner[m] for m in candidates})
            remaining = {frozenset(set(g) - {probe_id}) for g in result["groups"] if set(g) - {probe_id}}
            row["paths"] = [p for p in result["paths"] if probe_id in (p["source"], p["target"])]
            row["partition_unchanged"] = remaining == partitions
            if remaining != partitions:
                row.update(status="ambiguous", reason="probe_would_change_saved_partition")
            elif candidates and len(row["candidate_group_ids"]) == 1 and frozenset(candidates) in partitions:
                row.update(status="assigned", method="seated_basin", group_id=row["candidate_group_ids"][0])
            else:
                row["reason"] = "no_unique_existing_basin"
        evidence.append(row)
    return dict(schema_version=1, model=MODEL, annotations_only=True,
                drop_trials_executed=0, disturbance_trials_executed=0,
                settings=dict(barrier_mm=barrier_mm, max_angle_deg=max_angle_deg,
                              sampling_step_deg=step_deg), allocations=evidence)


def export_physical_allocation(run, catalogue, mesh_path, destination=None):
    """Write separate, reproducible annotations without modifying saved outputs."""
    run, catalogue = Path(run), Path(catalogue)
    def read(path):
        return json.loads(path.read_text(encoding="utf-8-sig"))
    data, manifest, config = (read(run / name) for name in ("groups.json", "manifest.json", "config.json"))
    annotations = read(catalogue)
    if (annotations["schema_version"] != 1 or
            annotations["label_namespace"] != "physical_test_sheet_column" or
            annotations["frame"] != {"quaternion_order": "xyzw", "rotation": "part_to_chute"}):
        raise ValueError("Incompatible physical orientation catalogue")
    inputs = allocation_inputs(run, catalogue, mesh_path)
    if inputs["mesh"] != manifest["mesh_source_sha256"]:
        raise ValueError("Physical allocation mesh differs from saved run")
    result = allocate_physical_poses(mesh_path, read(run / "group_members.json"), data["groups"],
                                    manifest["recognition"], annotations["poses"],
                                    alpha_deg=config["alpha_deg"], beta_deg=config["beta_deg"],
                                    barrier_mm=data["settings"]["barrier_mm"],
                                    max_angle_deg=data["settings"]["max_angle_deg"],
                                    step_deg=data["settings"]["sampling_step_deg"])
    result["input_sha256"] = inputs
    package = Path(__file__).parent
    result["engine_sha256"] = {name: hashlib.sha256((package / name).read_bytes()).hexdigest()
                               for name in ("physical_allocation.py", "grouping.py", "observed.py", "pose_matching.py")}
    if allocation_inputs(run, catalogue, mesh_path) != inputs:
        raise ValueError("Physical allocation inputs changed during processing")
    target = Path(destination) if destination else run / "physical_label_allocation.json"
    if ((target.resolve().is_relative_to(run.resolve()) and
         target.resolve() != (run / "physical_label_allocation.json").resolve()) or
            target.resolve() in {catalogue.resolve(), Path(mesh_path).resolve()}):
        raise ValueError("Allocation output must not overwrite saved run inputs")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return result
