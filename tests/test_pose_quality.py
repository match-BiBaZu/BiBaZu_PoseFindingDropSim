import copy
import hashlib
import json
import math
import sys
import subprocess
from pathlib import Path
import numpy as np
import pytest
import trimesh
from scipy.spatial.transform import Rotation
from dashas_drop_sim.pose_quality import analyze_pose_quality, quality_label, input_signature, sha256
from dashas_drop_sim.catalogue_quality import annotate_catalogue

def example(tmp_path):
    mesh = tmp_path / "Test.stl"
    trimesh.creation.box(extents=(40, 30, 2)).export(mesh)
    members = [{"id": "a", "count": 99, "incidence_percent": 99., "group_id": "G0", "quaternion_xyzw": Rotation.from_euler("y", 90, degrees=True).as_quat().tolist()},
               {"id": "b", "count": 1, "incidence_percent": 1., "group_id": "G0", "quaternion_xyzw": [0, 0, 0, 1]}]
    descriptor = {"symmetry_quaternions_xyzw": [[0, 0, 0, 1]]}
    return mesh, members, descriptor

def test_directed_stability_is_independent_of_incidence_and_membership_unchanged(tmp_path):
    mesh, members, descriptor = example(tmp_path)
    before = copy.deepcopy(members)
    report = analyze_pose_quality(mesh, members, descriptor, alpha_deg=0, beta_deg=0)
    assert members == before
    assert report["poses"]["a"]["escape_lift_mm"] == pytest.approx(math.hypot(20, 1) - 20, abs=1e-4)
    assert report["poses"]["b"]["escape_lift_mm"] == pytest.approx(math.hypot(20, 1) - 1, abs=1e-4)
    assert report["poses"]["a"]["stability"] == "low"
    assert report["poses"]["b"]["stability"] == "high"
    assert report["poses"]["a"]["escape_within_group"] is True
    for member in members:
        member["count"] = 0
        member["incidence_percent"] = 0
    other = analyze_pose_quality(mesh, members, descriptor, alpha_deg=0, beta_deg=0)
    for key in report["poses"]:
        assert other["poses"][key]["escape_lift_mm"] == report["poses"][key]["escape_lift_mm"]

@pytest.mark.parametrize("lift,label", [(0,"fragile"),(.1,"marginal"),(.5,"resistant"),(None,"unknown")])
def test_boundaries(lift,label):
    assert quality_label(lift) == label

def test_symmetry_and_long_paths_and_cancellation(tmp_path):
    mesh, members, descriptor = example(tmp_path)
    members[0]["quaternion_xyzw"] = [0,0,0,1]
    members[1]["quaternion_xyzw"] = Rotation.from_euler("y",120,degrees=True).as_quat().tolist()
    report = analyze_pose_quality(mesh,members,descriptor)
    assert report["poses"]["a"]["path_angle_deg"] == pytest.approx(120)
    members[1]["quaternion_xyzw"] = [0,0,1,0]
    descriptor["symmetry_quaternions_xyzw"].append([0,0,1,0])
    report = analyze_pose_quality(mesh,members,descriptor)
    assert all(p["stability"]=="unknown" for p in report["poses"].values())
    with pytest.raises(RuntimeError,match="cancelled"):
        analyze_pose_quality(mesh,members,descriptor,check_cancel=lambda: (_ for _ in ()).throw(RuntimeError("cancelled")))

def test_additive_catalogue_has_no_self_referential_checksum(tmp_path):
    mesh, members, descriptor = example(tmp_path)
    data = {"workpiece":"Test", "condition":"moving", "poses":members, "groups":[{"id":"G0","members":["a","b"]}],
            "settings":{"recognition":descriptor,"transverse_angle_deg":0,"longitudinal_angle_deg":0}}
    before=copy.deepcopy(data)
    annotated,report=annotate_catalogue(data,mesh,"config")
    assert data==before
    assert "source_sha256" not in annotated["pose_quality_assessment"]
    for original,current in zip(members,annotated["poses"]):
        assert original=={k:v for k,v in current.items() if k!="pose_quality"}
    assert annotated["groups"]==before["groups"]
    assert report["input_signature"] == input_signature(sha256(mesh),members,descriptor,0.,0.)

def test_geometry_core_does_not_import_physics():
    result=subprocess.run([sys.executable,"-c","import sys; import dashas_drop_sim.pose_quality; assert 'mujoco' not in sys.modules; assert 'dashas_drop_sim.physics' not in sys.modules"],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
