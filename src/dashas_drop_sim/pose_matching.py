"""Symmetry-aware recognition of generated and measured workpiece poses.

References are observed pose catalogues. Pose generation has no dependency on
transition graphs or downstream planning products. Rotations are xyzw,
part-to-chute, in the right-handed X/belt, Y/wall, Z/floor frame.
"""
from __future__ import annotations
from dataclasses import dataclass
import hashlib
import importlib
from importlib import metadata
import json
import math
from pathlib import Path
from typing import Literal, Sequence
from uuid import uuid4
import numpy as np
from scipy.cluster.hierarchy import fcluster, linkage
from scipy.spatial.distance import squareform
from scipy.spatial.transform import Rotation

MatchStatus = Literal["matched", "unmatched", "ambiguous"]
@dataclass(frozen=True, slots=True)
class PoseMatch:
    status: MatchStatus
    pose_id: int | str | None
    distance_deg: float | None

def _quaternions(values: Sequence[Sequence[float]] | np.ndarray) -> np.ndarray:
    quats = np.asarray(values, dtype=float)
    if quats.size == 0:
        return np.empty((0, 4), dtype=float)
    if quats.ndim != 2 or quats.shape[1] != 4 or not np.all(np.isfinite(quats)):
        raise ValueError("Expected finite xyzw quaternions with shape (n, 4).")
    norms = np.linalg.norm(quats, axis=1)
    if np.any(norms <= 1e-12):
        raise ValueError("A zero quaternion cannot describe an orientation.")
    return quats / norms[:, None]


def _quat(value: Sequence[float] | np.ndarray) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (4,):
        raise ValueError("Expected one xyzw quaternion with shape (4,).")
    quats = _quaternions(array[None, :])
    return quats[0]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_quat(quat: np.ndarray) -> tuple[float, ...]:
    signed = quat.copy()
    for value in signed[::-1]:
        if abs(float(value)) > 1e-12:
            if value < 0:
                signed *= -1
            break
    return tuple(float(x) for x in np.round(signed, 12))


class PoseResolver:
    """Detect part symmetry and cluster observed orientations without reference inputs."""
    def __init__(self, mesh_path: Path, *, original_mesh_path: Path | None = None,
                 cache_dir: Path | None = None, match_tolerance_deg=5.0,
                 ambiguity_margin_deg=1.0, cluster_tolerance_deg=5.0):
        for name, value, zero in (("match_tolerance_deg",match_tolerance_deg,False),
                                  ("ambiguity_margin_deg",ambiguity_margin_deg,True),
                                  ("cluster_tolerance_deg",cluster_tolerance_deg,False)):
            if not math.isfinite(value) or value<0 or (value==0 and not zero) or value>180:
                raise ValueError(f"{name} must be finite and between 0 and 180 degrees.")
        self.mesh_path=Path(mesh_path).expanduser().resolve()
        self.original_mesh_path=Path(original_mesh_path or mesh_path).expanduser().resolve()
        self.match_tolerance_deg=float(match_tolerance_deg)
        self.ambiguity_margin_deg=float(ambiguity_margin_deg)
        self.cluster_tolerance_deg=float(cluster_tolerance_deg)
        self.symmetry_available=False
        self.symmetry_symbol="C1"
        self._symmetry_quats=np.array([[0.,0.,0.,1.]])
        self._continuous_axis=None
        self._reference_quats=np.empty((0,4),dtype=float)
        self._reference_axes=np.empty((0,3),dtype=float)
        self._reference_ids=np.empty(0,dtype=str)
        self.known_pose_ids=()
        self.cache_hit=False
        try:
            geometry=importlib.import_module("chute_pose")
        except ModuleNotFoundError as exc:
            if exc.name!="chute_pose":raise
            return
        try:version=metadata.version("bibazu-chute-pose")
        except metadata.PackageNotFoundError:version="unknown"
        cache=None
        if cache_dir is not None:
            key=hashlib.sha256(json.dumps(dict(version=2,mesh_sha256=_sha256(self.mesh_path),package_version=version),sort_keys=True).encode()).hexdigest()
            cache=Path(cache_dir).expanduser().resolve()/f"{key}.json"
            if cache.is_file():
                try:
                    data=json.loads(cache.read_text(encoding="utf-8"))
                    quats=_quaternions(data["symmetry_quaternions_xyzw"])
                    axis=data["continuous_axis_part"]
                    if data["version"]!=2 or not len(quats):raise ValueError("Invalid symmetry cache")
                    self._symmetry_quats=quats
                    self._continuous_axis=None if axis is None else np.asarray(axis,dtype=float)
                    if self._continuous_axis is not None:
                        if self._continuous_axis.shape!=(3,) or not np.isfinite(self._continuous_axis).all() or np.linalg.norm(self._continuous_axis)<1e-12:raise ValueError("Invalid symmetry axis")
                        self._continuous_axis/=np.linalg.norm(self._continuous_axis)
                    self.symmetry_symbol=data["symmetry_symbol"]
                    self.symmetry_available=True
                    self.cache_hit=True
                    return
                except (OSError,KeyError,TypeError,ValueError):pass
        symmetry=geometry.detect_rotational_symmetry(self.mesh_path)
        self.symmetry_available=True
        self.symmetry_symbol=symmetry.symbol
        if symmetry.continuous_axis_part is not None:
            axis=np.asarray(symmetry.continuous_axis_part,dtype=float)
            self._continuous_axis=axis/np.linalg.norm(axis)
        else:
            self._symmetry_quats=Rotation.from_matrix(np.asarray([e.rotation_part_from_part for e in symmetry.elements])).as_quat()
        if cache is not None:
            cache.parent.mkdir(parents=True,exist_ok=True)
            temporary=cache.with_name(f"{uuid4().hex}.tmp")
            temporary.write_text(json.dumps(dict(version=2,**self.recognition_descriptor())),encoding="utf-8")
            temporary.replace(cache)

    def reference_quaternion(self, pose_id: int | str) -> tuple[float, float, float, float] | None:
        """One catalogue orientation for displaying a known observed pose."""
        indices = np.flatnonzero(self._reference_ids == pose_id)
        if len(indices) == 0:
            return None
        return tuple(float(value) for value in self._reference_quats[indices[0]])

    def recognition_descriptor(self) -> dict:
        """Portable symmetry and orientation conventions for exported observations."""
        return {
            "quaternion_order": "xyzw", "rotation": "part_to_chute",
            "symmetry_available": self.symmetry_available, "symmetry_symbol": self.symmetry_symbol,
            "symmetry_quaternions_xyzw": self._symmetry_quats.tolist(),
            "continuous_axis_part": None if self._continuous_axis is None else self._continuous_axis.tolist(),
            "tolerance_deg": self.cluster_tolerance_deg,
            "ambiguity_margin_deg": self.ambiguity_margin_deg,
            "symmetry_composition": "q_part_to_chute * q_symmetry_part",
        }

    def resolve(self, quat_xyzw: Sequence[float] | np.ndarray) -> PoseMatch:
        """Return one unambiguous observed pose, or retain the unknown result."""

        query = _quat(quat_xyzw)
        if len(self._reference_quats) == 0:
            return PoseMatch("unmatched", None, None)
        if self._continuous_axis is None:
            dots = np.clip(np.abs(self._reference_quats @ query), 0.0, 1.0)
            distances = np.degrees(2.0 * np.arccos(dots))
        else:
            axis = Rotation.from_quat(query).apply(self._continuous_axis)
            dots = np.clip(self._reference_axes @ axis, -1.0, 1.0)
            distances = np.degrees(np.arccos(dots))
        by_pose = [
            (float(np.min(distances[self._reference_ids == pose_id])), pose_id.item())
            for pose_id in np.unique(self._reference_ids)
        ]
        by_pose.sort()
        best_distance, best_pose = by_pose[0]
        if best_distance > self.match_tolerance_deg:
            return PoseMatch("unmatched", None, best_distance)
        if len(by_pose) > 1 and by_pose[1][0] - best_distance <= self.ambiguity_margin_deg:
            return PoseMatch("ambiguous", None, best_distance)
        return PoseMatch("matched", best_pose, best_distance)

    def cluster_unmatched(self, quaternions: Sequence[Sequence[float]] | np.ndarray) -> list[int]:
        """Complete-link classes under part symmetry, numbered deterministically.

        Every pair within a class is at most ``cluster_tolerance_deg`` apart.
        This keeps two distinct modes from merging through a chain of samples.
        Labels are ordered by a canonical orientation so input permutations do
        not renumber existing classes.
        """

        quats = _quaternions(quaternions)
        count = len(quats)
        if count == 0:
            return []
        if count == 1:
            return [0]
        if self._continuous_axis is None:
            symmetry = Rotation.from_quat(self._symmetry_quats)
            variants = [
                (Rotation.from_quat(quat) * symmetry).as_quat()
                for quat in quats
            ]
            keys = [min(_canonical_quat(variant) for variant in item) for item in variants]
        else:
            axes = Rotation.from_quat(quats).apply(self._continuous_axis)
            keys = [tuple(float(x) for x in np.round(axis, 12)) for axis in axes]

        order = sorted(range(count), key=lambda index: keys[index])
        sorted_quats = quats[order]
        distances = np.zeros((count, count), dtype=float)
        if self._continuous_axis is None:
            for row, original_index in enumerate(order):
                dots = np.abs(variants[original_index] @ sorted_quats.T)
                distances[row, :] = np.degrees(2.0 * np.arccos(np.clip(np.max(dots, axis=0), 0.0, 1.0)))
        else:
            sorted_axes = axes[order]
            distances = np.degrees(np.arccos(np.clip(sorted_axes @ sorted_axes.T, -1.0, 1.0)))
        # Floating point differences between equivalent symmetry variants can
        # make the two directed values differ slightly. Be conservative.
        distances = np.maximum(distances, distances.T)
        np.fill_diagonal(distances, 0.0)
        tree = linkage(squareform(distances, checks=False), method="complete")
        raw_labels = fcluster(tree, t=self.cluster_tolerance_deg, criterion="distance")
        first_key = {
            raw: min(keys[order[index]] for index in np.flatnonzero(raw_labels == raw))
            for raw in np.unique(raw_labels)
        }
        renumber = {raw: label for label, raw in enumerate(sorted(first_key, key=first_key.get))}
        labels = [0] * count
        for sorted_index, original_index in enumerate(order):
            labels[original_index] = renumber[int(raw_labels[sorted_index])]
        return labels
