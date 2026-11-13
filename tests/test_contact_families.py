import json
from pathlib import Path

import pytest
from scipy.spatial.transform import Rotation

from dashas_drop_sim.catalog import catalog_models
from dashas_drop_sim.grouping import analyze_groups
from dashas_drop_sim.rolling_basins import scoped_model


def fixture(part):
    return json.loads((Path(__file__).parent / "fixtures" / ("contact_" + part + ".json")).read_text())


def result(part, members=None):
    data = fixture(part)
    return analyze_groups(catalog_models()[part], data["members"] if members is None else members, data["recognition"])


def display_groups(part):
    data = fixture(part)
    display = {m["id"]: m["display_pose_id"] for m in data["members"]}
    return {frozenset(display[key] for key in group) for group in result(part)["groups"]}


def test_kk2i_rounded_side_and_end_families():
    groups = display_groups("Kk2i")
    assert frozenset(range(4, 28)) in groups
    assert frozenset((28, 29, 30)) in groups
    assert len(groups) == 6


def test_kl2a_requested_rolling_families():
    groups = display_groups("Kl2a")
    assert frozenset((3, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 17, 19)) in groups
    assert frozenset((16, 18, 21, 25)) in groups


def test_kk4a_only_tilted_pose_splits_from_previous_groups():
    data = fixture("Kk4a")
    tilted = next(m["id"] for m in data["members"] if m["display_pose_id"] == 120)
    expected = {frozenset(group) - {tilted} for group in data["previous_groups"]}
    expected.add(frozenset((tilted,)))
    assert {frozenset(g) for g in result("Kk4a")["groups"]} == expected


@pytest.mark.parametrize("part", ["Kk2i", "Kl2a", "Kk4a"])
def test_order_invariance_and_recognition_unchanged(part):
    data = fixture(part)
    before = json.dumps(data, sort_keys=True)
    a = result(part)
    b = result(part, list(reversed(data["members"])))
    assert a == b
    assert json.dumps(data, sort_keys=True) == before
    assert not a["energy_barrier_applied"]


def test_family_rule_is_geometry_based_not_pose_id_based():
    data = fixture("Kk2i")
    renamed = [{**m, "id": "observation_" + str(m["display_pose_id"])} for m in data["members"]]
    a = result("Kk2i", renamed)
    assert {"observation_" + str(i) for i in range(4, 28)} in [set(group) for group in a["groups"]]


def test_axis_tolerance_excludes_tilt_and_flat_support_stays_distinct():
    data = fixture("Kk2i")
    base = next(m for m in data["members"] if m["display_pose_id"] == 4)
    tilt = (Rotation.from_euler("y", 11, degrees=True) * Rotation.from_quat(base["quaternion_xyzw"])).as_quat().tolist()
    a = result("Kk2i", [base, {"id": "tilted_probe", "quaternion_xyzw": tilt}])
    assert ["tilted_probe"] in a["groups"]
    assert any(r["members"] == ["tilted_probe"] for r in a["review"])


def test_changed_mesh_falls_back_to_previous_engine(tmp_path):
    import trimesh
    mesh = tmp_path / "Kk2i.stl"
    trimesh.creation.box(extents=(10, 10, 10)).export(mesh)
    assert scoped_model(mesh) is None
    a = analyze_groups(mesh, [{"id": "a", "quaternion_xyzw": [0, 0, 0, 1]}], {})
    assert a["model"] == "observed-seated-geodesic-basins-v1"
