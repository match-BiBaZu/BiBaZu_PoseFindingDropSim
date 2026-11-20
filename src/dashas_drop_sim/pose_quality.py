"""Geometric pose stability, assessed after grouping without running physics."""
from __future__ import annotations
from collections import Counter
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

METHOD = "sampled_seated_escape_lift"
METHOD_VERSION = 1
ANGULAR_STEP_DEG = 0.1
THRESHOLDS_MM = {"fragile": 0.1, "resistant": 0.5}
STABILITY = {"resistant": "high", "marginal": "medium", "fragile": "low", "unknown": "unknown"}
_EQUIVALENT_ANGLE_RAD = 1e-6

class QualityCancelled(RuntimeError):
    pass

def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def input_signature(mesh_hash, members, descriptor, alpha_deg, beta_deg):
    payload = {"method": METHOD, "version": METHOD_VERSION, "mesh_sha256": mesh_hash,
               "recognition": descriptor, "alpha_deg": alpha_deg, "beta_deg": beta_deg,
               "poses": [{k: p.get(k) for k in ("id", "quaternion_xyzw", "group_id", "count", "incidence_percent", "frequency_percent")}
                         for p in sorted(members, key=lambda p: str(p["id"]))]}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()

def quality_label(lift_mm: float | None) -> str:
    if lift_mm is None:
        return "unknown"
    if not math.isfinite(lift_mm) or lift_mm < 0:
        raise ValueError("Escape lift must be finite and nonnegative.")
    # Avoid classifying an analytically exact threshold on its floating-point
    # rounding side. This tolerance is much smaller than mesh/sampling accuracy.
    if lift_mm < THRESHOLDS_MM["fragile"] - 1e-9:
        return "fragile"
    if lift_mm < THRESHOLDS_MM["resistant"] - 1e-9:
        return "marginal"
    return "resistant"


def _angle(config, key, default):
    value = config.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"Invalid {key} in source config.json.")
    return float(value)


def _symmetry(descriptor, mesh_path):
    symmetry_source = "saved_recognition"
    symbol = descriptor.get("symmetry_symbol")
    axis = descriptor.get("continuous_axis_part")
    raw_symmetry = descriptor.get("symmetry_quaternions_xyzw")
    if raw_symmetry is None:
        raise ValueError("Saved symmetry is required for pose stability.")
    raw = np.asarray(raw_symmetry, dtype=float)
    if raw.ndim != 2 or raw.shape[1] != 4 or not len(raw) or not np.isfinite(raw).all():
        raise ValueError("Invalid symmetry quaternions for pose quality.")
    norms = np.linalg.norm(raw, axis=1)
    if np.any(norms < 1e-12):
        raise ValueError("Invalid symmetry quaternion length for pose quality.")
    # Identity is always available, including descriptors that omit it.
    symmetries = Rotation.from_quat(np.vstack(([0, 0, 0, 1], raw / norms[:, None]))).as_matrix()
    if axis is not None:
        axis = np.asarray(axis, dtype=float)
        if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-12:
            raise ValueError("Invalid continuous symmetry axis for pose quality.")
        axis = axis / np.linalg.norm(axis)
    return symmetries, axis, symmetry_source, symbol


def _relative_path(first, second, symmetries, continuous_axis):
    if continuous_axis is not None:
        u, v = first @ continuous_axis, second @ continuous_axis
        cross = np.cross(u, v)
        sine = float(np.linalg.norm(cross))
        cosine = float(np.clip(u @ v, -1, 1))
        angle = math.atan2(sine, cosine)
        if angle < _EQUIVALENT_ANGLE_RAD:
            return np.zeros(3)
        if sine < 1e-10:
            cross = np.cross(u, np.eye(3)[np.argmin(np.abs(u))])
        return cross / np.linalg.norm(cross) * angle
    candidates = Rotation.from_matrix((second @ symmetries) @ first.T).as_rotvec()
    return min(candidates, key=lambda rv: (float(np.linalg.norm(rv)), tuple(rv)))


def _sample_path(vertices, first, rotvec, gravity, check_cancel):
    """Return both directed barriers and peak fractions using bounded memory."""
    angle = float(np.degrees(np.linalg.norm(rotvec)))
    count = max(2, math.ceil(angle / ANGULAR_STEP_DEG) + 1)
    fractions = np.linspace(0.0, 1.0, count)
    # Two support-function arrays, each capped at about 4 MiB. Computing only
    # the wall/floor coordinates avoids materializing samples x vertices x 3.
    chunk = max(1, min(128, 524288 // max(1, len(vertices))))
    heights = np.empty(count)
    for start in range(0, count, chunk):
        check_cancel()
        rotation = Rotation.from_rotvec(fractions[start:start + chunk, None] * rotvec).as_matrix() @ first
        min_y = np.einsum("sj,vj->sv", rotation[:, 1, :], vertices, optimize=False).min(axis=1)
        min_z = np.einsum("sj,vj->sv", rotation[:, 2, :], vertices, optimize=False).min(axis=1)
        heights[start:start + chunk] = gravity[1] * min_y + gravity[2] * min_z
    if not np.isfinite(heights).all():
        raise ValueError("Non-finite seated height encountered during pose-quality analysis.")
    peak = int(np.argmax(heights))
    return (max(0.0, float(heights[peak] - heights[0])),
            max(0.0, float(heights[peak] - heights[-1])),
            angle * float(fractions[peak]), angle * (1.0 - float(fractions[peak])))



def analyze_pose_quality(mesh_path, members, descriptor, *, alpha_deg=45, beta_deg=0,
                         evidence=None, progress=lambda message: None, check_cancel=lambda: None):
    """Assess all recorded orientations; groups and observations are input-only."""
    check_cancel()
    members = deepcopy(members)
    mesh_path = Path(mesh_path)
    mesh_hash = sha256(mesh_path)
    alpha = _angle({"alpha_deg": alpha_deg}, "alpha_deg", 45)
    beta = _angle({"beta_deg": beta_deg}, "beta_deg", 0)
    mesh = trimesh.load_mesh(mesh_path, process=True)
    if not mesh.is_watertight or not mesh.is_volume:
        raise ValueError("Stability requires a closed solid mesh in millimetres.")
    vertices = np.asarray(mesh.convex_hull.vertices) - np.asarray(mesh.center_mass)
    ordered = sorted(members, key=lambda p: str(p["id"]))
    ids = [str(p["id"]) for p in ordered]
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate source pose IDs.")
    groups = {str(p["id"]): str(p["group_id"]) for p in members if p.get("group_id") is not None}
    poses = {str(p["id"]): {"label": "unknown", "stability": "unknown", "escape_lift_mm": None,
        "escape_target_source_id": None, "escape_within_group": None, "source_group_id": groups.get(str(p["id"])),
        "path_angle_deg": None, "peak_angle_deg": None, "path_origin": None,
        "observed_count": p.get("count"), "observed_frequency_percent": p.get("incidence_percent", p.get("frequency_percent")),
        "reason": "No symmetry-distinct recorded target pose is available."} for p in members}
    try:
        symmetry, axis, symmetry_source, symbol = _symmetry(descriptor, mesh_path)
    except ValueError as error:
        if descriptor.get("symmetry_quaternions_xyzw") is not None:
            raise
        symmetry = axis = None
        symmetry_source, symbol = "unavailable", None
        for entry in poses.values():
            entry["reason"] = str(error)
    rotations = {}
    for p in ordered:
        q = np.asarray(p["quaternion_xyzw"], dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
            raise ValueError("Invalid pose quaternion.")
        rotations[str(p["id"])] = Rotation.from_quat(q).as_matrix()
    gravity = (Rotation.from_euler("y", beta, degrees=True) * Rotation.from_euler("x", alpha, degrees=True)).inv().apply([0, 0, -1.])
    saved = {}
    if (evidence or {}).get("model") == "observed-seated-geodesic-basins-v1" and 0 < evidence.get("sampling_step_deg", 999) <= .1:
        for p in evidence.get("paths", []):
            values = [p.get(k) for k in ("angle_deg", "forward_barrier_mm", "reverse_barrier_mm")]
            if all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v >= 0 for v in values):
                saved[str(p["source"]), str(p["target"])] = p
    reused = calculated = equivalent = 0
    def consider(a, b, lift, angle, peak, origin):
        entry = poses[a]
        previous = entry["escape_lift_mm"]
        if previous is not None and (lift, b) >= (previous, entry["escape_target_source_id"]):
            return
        within = groups[a] == groups[b] if a in groups and b in groups else None
        label = quality_label(lift)
        entry.update(label=label, stability=STABILITY[label], escape_lift_mm=float(lift),
            escape_target_source_id=b, escape_within_group=within, path_angle_deg=float(angle),
            peak_angle_deg=peak, path_origin=origin,
            reason="Movement stays within the existing pose group (rocking); this does not classify the whole group."
            if within else "Sampled geometric escape to another recorded pose; not a dynamic stability guarantee.")
    if symmetry is not None:
        for index, first in enumerate(ids):
            for second in ids[index + 1:]:
                check_cancel()
                rv = _relative_path(rotations[first], rotations[second], symmetry, axis)
                angle = float(np.degrees(np.linalg.norm(rv)))
                if np.linalg.norm(rv) <= _EQUIVALENT_ANGLE_RAD:
                    equivalent += 1
                    continue
                path = saved.get((first, second))
                reverse = False
                if path is None:
                    path, reverse = saved.get((second, first)), True
                if path is not None and abs(path["angle_deg"] - angle) <= 1e-6:
                    forward, backward = path["forward_barrier_mm"], path["reverse_barrier_mm"]
                    if reverse:
                        forward, backward = backward, forward
                    peak_a = peak_b = None
                    origin = "verified_saved_path"
                    reused += 1
                else:
                    forward, backward, peak_a, peak_b = _sample_path(vertices, rotations[first], rv, gravity, check_cancel)
                    origin = "calculated_path"
                    calculated += 1
                consider(first, second, forward, angle, peak_a, origin)
                consider(second, first, backward, angle, peak_b, origin)
            progress(f"Stability checked {index + 1}/{len(ids)} poses")
    check_cancel()
    if sha256(mesh_path) != mesh_hash:
        raise ValueError("Mesh changed during stability assessment.")
    counts = Counter(p["label"] for p in poses.values())
    return {"schema_version": 1, "method": METHOD, "method_version": METHOD_VERSION,
        "producer": "dashas_drop_sim", "engine_sha256": sha256(__file__),
        "input_signature": input_signature(mesh_hash, members, descriptor, alpha, beta),
        "mesh_sha256": mesh_hash, "angular_step_deg": .1, "thresholds_mm": dict(THRESHOLDS_MM),
        "alpha_deg": alpha, "beta_deg": beta, "symmetry_source": symmetry_source, "symmetry_symbol": symbol,
        "frequency_independent": True, "automatic_filtering": False, "uniform_density_assumed": True,
        "interpretation": "Sampled geometric tipping resistance, not dynamic survival or a proven minimum-energy path. Group membership and observations are unchanged.",
        "path_counts": {"reused": reused, "calculated": calculated, "symmetry_equivalent": equivalent},
        "label_counts": {k: counts[k] for k in STABILITY},
        "stability_counts": {STABILITY[k]: counts[k] for k in STABILITY}, "poses": poses}
