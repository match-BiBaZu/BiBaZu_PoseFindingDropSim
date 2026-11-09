"""Direct observation matching; source IDs are never geometric catalog indices."""
from __future__ import annotations

import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

from .pose_matching import PoseResolver, _quaternions, _sha256


class ObservedPoseResolver(PoseResolver):
    def __init__(self, mesh_path: Path, catalog_path: Path | None = None, *, data: dict | None = None):
        self.data = data if data is not None else json.loads(Path(catalog_path).read_text(encoding="utf-8-sig"))
        if self.data.get("schema_version", 1) != 1 or not isinstance(self.data.get("poses"), list):
            raise ValueError("Expected a version 1 observed poses catalog.")
        expected = self.data.get("geometry", {}).get("stl_sha256")
        if expected and expected != _sha256(mesh_path):
            raise ValueError("Observed catalog STL SHA-256 differs from the selected mesh.")
        self.mesh_path = self.original_mesh_path = Path(mesh_path)
        self.cache_hit = False
        descriptor = self.data.get("recognition", {})
        if descriptor.get("quaternion_order", "xyzw") != "xyzw" or descriptor.get("rotation", "part_to_chute") != "part_to_chute":
            raise ValueError("Observed catalog requires xyzw part-to-chute rotations.")
        self.match_tolerance_deg = self.cluster_tolerance_deg = float(descriptor.get("tolerance_deg", 5))
        self.ambiguity_margin_deg = float(descriptor.get("ambiguity_margin_deg", 1))
        if not 0 < self.match_tolerance_deg <= 180 or not 0 <= self.ambiguity_margin_deg <= 180:
            raise ValueError("Invalid observation recognition tolerances.")
        self._symmetry_quats = _quaternions(descriptor.get("symmetry_quaternions_xyzw", [[0, 0, 0, 1]]))
        if not len(self._symmetry_quats):
            raise ValueError("Observed symmetry requires at least identity.")
        axis = descriptor.get("continuous_axis_part")
        self._continuous_axis = None if axis is None else np.asarray(axis, dtype=float)
        if axis is not None:
            if self._continuous_axis.shape != (3,) or not np.all(np.isfinite(self._continuous_axis)) or np.linalg.norm(self._continuous_axis) < 1e-12:
                raise ValueError("Invalid continuous symmetry axis.")
            self._continuous_axis = self._continuous_axis / np.linalg.norm(self._continuous_axis)
        self.symmetry_symbol = descriptor.get("symmetry_symbol", "C1")
        self.symmetry_available = bool(descriptor.get("symmetry_available", True))
        ids, quats, seen = [], [], set()
        for pose in self.data["poses"]:
            key = str(pose["id"])
            if not key or key in seen:
                raise ValueError("Observed pose IDs must be unique and nonempty.")
            seen.add(key)
            references = _quaternions(pose.get("reference_quaternions_xyzw", [pose["quaternion_xyzw"]]))
            if not len(references):
                raise ValueError("An observed pose needs a reference orientation.")
            if self._continuous_axis is None:
                # Match all measured references, including their finite symmetry variants.
                references = (Rotation.from_quat(references).as_matrix()[:, None]
                              @ Rotation.from_quat(self._symmetry_quats).as_matrix()[None]).reshape(-1, 3, 3)
                references = Rotation.from_matrix(references).as_quat()
            quats.extend(references)
            ids.extend([key] * len(references))
        self.known_pose_ids = tuple(sorted(seen))
        self._reference_quats = np.asarray(quats, dtype=float).reshape(-1, 4)
        self._reference_ids = np.asarray(ids, dtype=str)
        self._reference_axes = (np.empty((0, 3)) if self._continuous_axis is None else
                                Rotation.from_quat(self._reference_quats).apply(self._continuous_axis))

    def grouped(self, mapping: dict[str, str]) -> "ObservedPoseResolver":
        import copy
        result = copy.copy(self)
        result._reference_ids = np.asarray([mapping[str(key)] for key in self._reference_ids])
        result.known_pose_ids = tuple(sorted(set(mapping.values())))
        return result


def observations_from_trials(mesh_path, rows, recognition, *, baseline=None):
    """Rebuild the published observed catalogue without trusting earlier group keys.

    Catalogue anchors take precedence when supplied. New classes use the first
    settled trial in trial-ID order and keep every measured orientation.
    """
    from .runner import SETTLED_STATUSES
    baseline = baseline or {"poses": [], "recognition": recognition}
    resolver = ObservedPoseResolver(Path(mesh_path), data=baseline)
    actual = resolver.recognition_descriptor()
    if (any(actual.get(k) != recognition.get(k) for k in
            ("tolerance_deg", "ambiguity_margin_deg", "symmetry_symbol", "quaternion_order", "rotation"))
            or not np.allclose(actual["symmetry_quaternions_xyzw"], recognition["symmetry_quaternions_xyzw"], atol=1e-12)
            or actual["continuous_axis_part"] != recognition.get("continuous_axis_part")):
        raise ValueError("Observed anchors and saved trials use different recognition settings.")
    settled = sorted((r for r in rows if r["status"] in SETTLED_STATUSES), key=lambda r: r["trial"])
    assignments, unknown = {}, []
    for row in settled:
        match = resolver.resolve(row["final_quat_xyzw"])
        if match.status == "matched":
            assignments[row["trial"]] = (str(match.pose_id), "matched")
        elif match.status == "ambiguous":
            assignments[row["trial"]] = (None, "ambiguous")
        else:
            unknown.append(row)
    labels = resolver.cluster_unmatched([r["final_quat_xyzw"] for r in unknown])
    originals = {str(p["id"]): dict(p) for p in baseline["poses"]}
    next_id = 1
    for label in sorted(set(labels)):
        while (key := f"{Path(mesh_path).stem}_{next_id:04d}") in originals:
            next_id += 1
        next_id += 1
        members = [row for row, value in zip(unknown, labels) if value == label]
        first = members[0]
        originals[key] = {"id": key, "quaternion_xyzw": first["final_quat_xyzw"],
                          "position_com_chute_mm": first["final_pos_chute_mm"],
                          "reference_quaternions_xyzw": [], "representative_trial": first["trial"]}
        for row in members:
            assignments[row["trial"]] = (key, "new_observation")
    poses = []
    for key, original in sorted(originals.items()):
        members = [r for r in settled if assignments[r["trial"]][0] == key]
        references = {tuple(q) for q in original.get("reference_quaternions_xyzw", [original["quaternion_xyzw"]])}
        references.update(tuple(r["final_quat_xyzw"]) for r in members)
        original.update(count=len(members), trial_ids=[r["trial"] for r in members],
                        reference_quaternions_xyzw=[list(q) for q in sorted(references)],
                        source_trial=members[0]["trial"] if members else None)
        poses.append(original)
    annotated = []
    for row in sorted(rows, key=lambda r: r["trial"]):
        key, status = assignments.get(row["trial"], (None, "not_evaluated"))
        annotated.append({**row, "source_pose_id": key, "source_match_status": status})
    return {"schema_version": 1, "workpiece": Path(mesh_path).stem,
            "geometry": {"stl_sha256": _sha256(Path(mesh_path))},
            "recognition": recognition, "completed_trials": len(rows), "poses": poses}, annotated
