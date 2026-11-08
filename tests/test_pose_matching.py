"""Observed pose mapping must preserve unknown modes and catalogue equivalences."""

from __future__ import annotations

import json
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dashas_drop_sim.pose_matching import PoseResolver


def _fake_chute_pose(monkeypatch, *, continuous_axis=None):
    module = ModuleType("chute_pose")
    def forbidden(*_a, **_kw):
        raise AssertionError("Pose catalogues must not be generated for recognition")
    module.build_pose_catalog = forbidden
    z_half_turn = Rotation.from_euler("z", 180, degrees=True).as_matrix()
    module.detect_rotational_symmetry = lambda _path, **_kwargs: SimpleNamespace(
        symbol="Cinf" if continuous_axis is not None else "C2",
        continuous_axis_part=continuous_axis,
        elements=tuple(
            SimpleNamespace(rotation_part_from_part=matrix)
            for matrix in (np.eye(3), z_half_turn)
        ),
    )
    monkeypatch.setitem(sys.modules, "chute_pose", module)


def test_unknown_pose_clusters_are_complete_link_and_order_stable(monkeypatch, tmp_path):
    _fake_chute_pose(monkeypatch)
    mesh = tmp_path / "part.stl"
    mesh.write_bytes(b"mesh")
    resolver = PoseResolver(mesh, cluster_tolerance_deg=5)
    angles = [0, 4, 8, 180]
    quats = Rotation.from_euler("z", [[angle] for angle in angles], degrees=True).as_quat()

    assert resolver.resolve(quats[0]).status == "unmatched"
    labels = resolver.cluster_unmatched(quats)
    assert labels[0] == labels[3]
    assert labels[2] != labels[0]
    assert set(labels) == {0, 1}
    permutation = [3, 2, 0, 1]
    permuted = resolver.cluster_unmatched(quats[permutation])
    assert permuted == [labels[index] for index in permutation]


def test_continuous_symmetry_ignores_spin_around_part_axis(monkeypatch, tmp_path):
    _fake_chute_pose(monkeypatch, continuous_axis=(0, 0, 1))
    mesh = tmp_path / "part.stl"
    mesh.write_bytes(b"mesh")
    resolver = PoseResolver(mesh)
    quats = Rotation.from_euler("z", [[0], [70], [140]], degrees=True).as_quat()
    tilted = Rotation.from_euler("x", 20, degrees=True).as_quat()

    labels = resolver.cluster_unmatched(np.vstack((quats, tilted)))
    assert labels[0] == labels[1] == labels[2]
    assert labels[3] != labels[0]


def test_invalid_quaternion_is_rejected(monkeypatch, tmp_path):
    _fake_chute_pose(monkeypatch)
    mesh = tmp_path / "part.stl"
    mesh.write_bytes(b"mesh")
    resolver = PoseResolver(mesh)

    with pytest.raises(ValueError, match="zero quaternion"):
        resolver.resolve([0, 0, 0, 0])


def test_symmetry_cache_contains_no_pose_catalogue(monkeypatch, tmp_path):
    _fake_chute_pose(monkeypatch)
    mesh = tmp_path / "part.stl"
    mesh.write_bytes(b"same mesh")
    cache = tmp_path / "pose_cache"
    first = PoseResolver(mesh, cache_dir=cache)
    assert not first.cache_hit and first.known_pose_ids == ()
    def forbidden(*_a, **_kw):
        raise AssertionError("Geometry catalogue generation must never be called")
    monkeypatch.setattr(sys.modules["chute_pose"], "build_pose_catalog", forbidden)
    monkeypatch.setattr(sys.modules["chute_pose"], "detect_rotational_symmetry", forbidden)
    second = PoseResolver(mesh, cache_dir=cache)
    assert second.cache_hit and second.known_pose_ids == ()
    assert second.recognition_descriptor() == first.recognition_descriptor()
    assert second.resolve([0,0,0,1]).status == "unmatched"
    data = json.loads(next(cache.glob("*.json")).read_text())
    assert "poses" not in data and "nodes" not in data


def test_pose_generation_never_calls_catalogue_builder(monkeypatch, tmp_path):
    _fake_chute_pose(monkeypatch)
    def forbidden(*_a, **_kw):
        raise AssertionError("Pose generation must be independent of downstream products")
    monkeypatch.setattr(sys.modules["chute_pose"], "build_pose_catalog", forbidden)
    mesh = tmp_path / "part.stl"
    mesh.write_bytes(b"mesh")
    resolver = PoseResolver(mesh)
    assert resolver.symmetry_available and resolver.known_pose_ids == ()
    assert resolver.cluster_unmatched([[0,0,0,1],[0,0,0,-1]]) == [0,0]
