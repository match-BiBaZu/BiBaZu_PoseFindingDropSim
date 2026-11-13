"""Persistent seated-energy basins, restricted to six verified catalogue solids.

Edges describe sampled local rotation paths, not arbitrary proximity merges.
Persistence uses the actual escape saddle above each minimum; assignment uses
the maximum absolute height along a path, never a sum of small uphill steps.
"""
from __future__ import annotations

import hashlib
import heapq
import math
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from .physics import _chute_rotation

MODEL_VERSION = "scoped-persistent-seated-energy-basins-v2"
SCOPED_MESHES = {
    "Kk2i": "ec6ebb3923fcab8e0a02546aac25cb13f893584b551a81ced4315e1059195ef7",
    "Kk4a": "8b9e742443c8153aca1f5afd3e68562d5cecd7ac9af010f2ede3b0162fa200c1",
    "Kl4i": "3b1586a009406e6108f590bbe6316004d9a6a18944d92b64d5f2e5acdff6e831",
    "Kl2a": "f874e82aea3486c1887ad267db840f2cd2899527241407e5f142bc8bcf70a5fe",
    "Kf2a": "cd1bb04dda24ff3aa100827f1042e6abb2459c6bd1301fba1be041b49d1ab30c",
    "Kf4i": "56720a15e28abcf47d6705716e1033a8146c4970e212b7d3f3cd2f33ab3e64ca",
}


def scoped_model(mesh_path):
    path = Path(mesh_path)
    identity = next((key for key in SCOPED_MESHES if key.casefold() == path.stem.casefold()), None)
    if identity is None:
        return None
    return identity if hashlib.sha256(path.read_bytes()).hexdigest() == SCOPED_MESHES[identity] else None


def energy_basins(heights, edges, observations, barrier_mm):
    """Return deterministic groups, ambiguity, minima and minimax evidence.

    ``observations`` maps stable pose IDs to graph nodes. Internal path samples
    participate in the energy landscape even when no drop landed exactly there.
    """
    heights = [float(h) for h in heights]
    if barrier_mm < 0 or not math.isfinite(barrier_mm) or not all(map(math.isfinite, heights)):
        raise ValueError("Expected finite heights and a nonnegative finite barrier")
    size = len(heights)
    parent = list(range(size))
    birth = list(range(size))
    adjacency = [[] for _ in heights]
    weighted = []
    for a, b in edges:
        peak = max(heights[a], heights[b])
        weighted.append((peak, min(a, b), max(a, b)))
        adjacency[a].append((b, peak))
        adjacency[b].append((a, peak))

    def find(node):
        while parent[node] != node:
            parent[node] = parent[parent[node]]
            node = parent[node]
        return node

    minima = []
    cores = []
    for peak, a, b in sorted(weighted):
        a, b = find(a), find(b)
        if a == b:
            continue
        if (heights[birth[b]], birth[b]) < (heights[birth[a]], birth[a]):
            a, b = b, a
        losing = birth[b]
        prominence = peak - heights[losing]
        if prominence > barrier_mm + 1e-9:
            cores.append(losing)
            minima.append({"node": losing, "height_mm": heights[losing],
                           "escape_saddle_mm": peak, "escape_barrier_mm": prominence})
        parent[b] = a
    for root in sorted({find(node) for node in range(size)}):
        core = birth[root]
        cores.append(core)
        minima.append({"node": core, "height_mm": heights[core],
                       "escape_saddle_mm": None, "escape_barrier_mm": None})
    cores.sort()
    reachable = {key: [] for key in sorted(observations)}
    for core in cores:
        costs = [math.inf] * size
        costs[core] = heights[core]
        pending = [(costs[core], core)]
        while pending:
            cost, node = heapq.heappop(pending)
            if cost != costs[node]:
                continue
            for target, peak in adjacency[node]:
                candidate = max(cost, peak)
                if candidate < costs[target]:
                    costs[target] = candidate
                    heapq.heappush(pending, (candidate, target))
        for key, node in sorted(observations.items()):
            if costs[node] - heights[node] <= barrier_mm + 1e-9:
                reachable[key].append({"minimum_node": core, "minimax_height_mm": costs[node],
                                       "required_lift_mm": max(0., costs[node] - heights[node])})
    buckets, review = {}, []
    for key, destinations in reachable.items():
        if len(destinations) == 1:
            buckets.setdefault(destinations[0]["minimum_node"], []).append(key)
        else:
            buckets["review:" + key] = [key]
            review.append({"members": [key], "reason": "ambiguous_energy_destination" if destinations else "no_energy_destination",
                           "destinations": destinations})
    return sorted(buckets.values(), key=tuple), review, sorted(minima, key=lambda m: m["node"]), reachable


def analyze_rolling(mesh_path, members, descriptor, *, identity, alpha_deg, beta_deg,
                    barrier_mm, max_angle_deg, step_deg):
    from .grouping import relative_path
    mesh = trimesh.load_mesh(mesh_path, process=True)
    if not mesh.is_watertight or not mesh.is_volume:
        raise ValueError("Grouping requires a closed solid mesh in mm.")
    vertices = np.asarray(mesh.convex_hull.vertices) - np.asarray(mesh.center_mass)
    gravity = _chute_rotation(alpha_deg, beta_deg).inv().apply([0, 0, -9.81])

    def seated(rotations):
        points = np.einsum("sij,vj->svi", rotations, vertices)
        return (gravity[1] * points[:, :, 1].min(axis=1)
                + gravity[2] * points[:, :, 2].min(axis=1)) / np.linalg.norm(gravity)

    ordered = sorted(members, key=lambda m: m["id"])
    rotations = [Rotation.from_quat(m["quaternion_xyzw"]).as_matrix() for m in ordered]
    heights = seated(np.asarray(rotations)).tolist() if ordered else []
    observations = {m["id"]: i for i, m in enumerate(ordered)}
    edges, paths = [], []
    for i, first in enumerate(ordered):
        for j in range(i + 1, len(ordered)):
            second = ordered[j]
            rv = relative_path(first["quaternion_xyzw"], second["quaternion_xyzw"], descriptor)
            angle = float(np.degrees(np.linalg.norm(rv)))
            path = {"source": first["id"], "target": second["id"], "angle_deg": angle,
                    "forward_barrier_mm": None, "reverse_barrier_mm": None}
            if angle <= max_angle_deg + 1e-9:
                fractions = np.linspace(0, 1, max(2, math.ceil(angle / step_deg) + 1))
                sampled = seated(Rotation.from_rotvec(fractions[:, None] * rv).as_matrix() @ rotations[i])
                nodes = [i, *range(len(heights), len(heights) + len(sampled) - 2), j]
                heights.extend(sampled[1:-1].tolist())
                edges.extend(zip(nodes, nodes[1:]))
                path.update(forward_barrier_mm=max(0., float(sampled.max() - sampled[0])),
                            reverse_barrier_mm=max(0., float(sampled.max() - sampled[-1])),
                            sample_nodes=nodes, sampled_heights_mm=sampled.tolist())
            paths.append(path)
    groups, review, minima, destinations = energy_basins(heights, edges, observations, barrier_mm)
    return {"model": MODEL_VERSION, "catalogue_identity": identity, "mesh_sha256": SCOPED_MESHES[identity],
            "barrier_mm": barrier_mm, "max_angle_deg": max_angle_deg, "angle_limit_scope": "local_path",
            "sampling_step_deg": step_deg, "paths": paths, "groups": groups, "review": review,
            "minima": minima, "observed_heights_mm": {k: heights[n] for k, n in observations.items()},
            "destinations": destinations, "graph_nodes": len(heights), "graph_edges": len(edges),
            "interpretation": "Sampled seated local paths; persistent minima and minimax heights are geometric estimates, not minimum-action proofs."}
