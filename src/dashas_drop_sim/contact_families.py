"""Simple support families for three pinned extruded, rounded catalogue solids.

This is a geometric contact classification, not an assertion of zero energy
barrier or additional rotational symmetry. Recognition remains untouched.
"""
from __future__ import annotations

import math
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from .physics import _chute_rotation
from .rolling_basins import SCOPED_MESHES

CONTACT_MODEL_VERSION = "scoped-rounded-support-families-v3"
CONTACT_MESHES = {key: SCOPED_MESHES[key] for key in ("Kk2i", "Kk4a", "Kl2a")}
PROFILES = {
    "Kk2i": {"directed_axis": False, "broad_cut_support": True, "merge_belt_end_azimuth": False},
    "Kl2a": {"directed_axis": False, "broad_cut_support": True, "merge_belt_end_azimuth": True},
    "Kk4a": {"directed_axis": True, "broad_cut_support": False, "merge_belt_end_azimuth": True},
}
AXIS_TOLERANCE_DEG = 10.
SUPPORT_NORMAL_TOLERANCE_DEG = 5.
WIDE_FACET_AREA_RATIO = 2.


def analyze_contact_families(mesh_path, members, descriptor, *, identity, alpha_deg=45,
                             beta_deg=0, barrier_mm=.5, max_angle_deg=60, step_deg=.1):
    mesh = trimesh.load_mesh(mesh_path, process=True)
    if not mesh.is_watertight or not mesh.is_volume:
        raise ValueError("Grouping requires a closed solid mesh in mm.")
    profile = PROFILES[identity]
    hull = mesh.convex_hull
    vertices = np.asarray(hull.vertices) - np.asarray(mesh.center_mass)
    normals = np.asarray(hull.facets_normal)
    areas = np.asarray([hull.area_faces[facet].sum() for facet in hull.facets])
    # All three pinned catalogue meshes are extruded along source Z. Hull
    # planes spanning a cut are wider than the cylindrical tessellation strips.
    side = np.abs(normals[:, 2]) < 1e-6
    typical_strip_area = float(np.median(areas[side])) if side.any() else 0.
    broad = side & (areas >= WIDE_FACET_AREA_RATIO * typical_strip_area) if typical_strip_area else np.zeros(len(areas), dtype=bool)
    broad_normals = normals[broad] if profile["broad_cut_support"] else np.empty((0, 3))
    gravity = _chute_rotation(alpha_deg, beta_deg).inv().apply([0, 0, -9.81])
    buckets, evidence, review, heights = {}, {}, [], {}
    for member in sorted(members, key=lambda m: m["id"]):
        key = member["id"]
        rotation = Rotation.from_quat(member["quaternion_xyzw"]).as_matrix()
        axis = rotation[:, 2]
        dominant = int(np.argmax(np.abs(axis)))
        angle = math.degrees(math.acos(float(np.clip(abs(axis[dominant]), -1, 1))))
        supports = []
        support_details = []
        for chute_normal in ([0, -1, 0], [0, 0, -1]):
            normal_part = rotation.T @ np.asarray(chute_normal)
            dots = broad_normals @ normal_part
            best = float(dots.max()) if len(dots) else -1.
            is_broad = best >= math.cos(math.radians(SUPPORT_NORMAL_TOLERANCE_DEG))
            supports.append("cut" if is_broad else "rounded")
            support_details.append({"outward_normal_part": normal_part.tolist(), "broad_cut_support": is_broad,
                                    "closest_broad_normal_angle_deg": math.degrees(math.acos(float(np.clip(best, -1, 1)))) if len(dots) else None})
        axis_name = "XYZ"[dominant]
        sign = ("+" if axis[dominant] >= 0 else "-") if profile["directed_axis"] else "unsigned"
        if angle > AXIS_TOLERANCE_DEG + 1e-9:
            signature = ("oblique", key)
            review.append({"members": [key], "reason": "axis_outside_contact_family", "axis_alignment_deg": angle})
        elif dominant == 0:
            signature = (axis_name, sign, *supports)
        elif dominant == 2:
            signature = (axis_name, sign, "end_on_belt", "any_azimuth" if profile["merge_belt_end_azimuth"] else supports[0])
        else:
            signature = (axis_name, sign, "end_on_wall", supports[1])
        buckets.setdefault(signature, []).append(key)
        points = vertices @ rotation.T
        heights[key] = float((gravity[1] * points[:, 1].min() + gravity[2] * points[:, 2].min()) / np.linalg.norm(gravity))
        evidence[key] = {"extrusion_axis_chute": axis.tolist(), "axis_alignment_deg": angle,
                         "signature": list(signature), "wall_and_belt_support": support_details}
    groups = sorted(buckets.values(), key=tuple)
    return {"model": CONTACT_MODEL_VERSION, "catalogue_identity": identity, "mesh_sha256": CONTACT_MESHES[identity],
            "groups": groups, "review": review, "paths": [], "observed_heights_mm": heights,
            "contact_evidence": evidence, "profile": profile, "axis_part": [0, 0, 1],
            "axis_tolerance_deg": AXIS_TOLERANCE_DEG, "support_normal_tolerance_deg": SUPPORT_NORMAL_TOLERANCE_DEG,
            "wide_facet_area_ratio": WIDE_FACET_AREA_RATIO, "typical_strip_area_mm2": typical_strip_area,
            "broad_support_normals_part": broad_normals.tolist(), "broad_support_areas_mm2": areas[broad].tolist() if profile["broad_cut_support"] else [],
            "barrier_mm": barrier_mm, "max_angle_deg": max_angle_deg, "sampling_step_deg": step_deg,
            "energy_barrier_applied": False,
            "interpretation": "Rounded-side roll phase is ignored within a support family; broad cut support and end direction distinguish families. This classification does not prove an energy barrier or change recognition symmetry."}
