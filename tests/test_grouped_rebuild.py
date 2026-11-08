"""Saved-pose reconstruction, original data preservation and published fixtures."""
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dashas_drop_sim.catalog import catalog_models
from dashas_drop_sim.config import RunConfig
from dashas_drop_sim.grouping import analyze_groups
from dashas_drop_sim.observed import ObservedPoseResolver, observations_from_trials


def descriptor(mesh):
    return ObservedPoseResolver(mesh, data={'poses': []}).recognition_descriptor()


def trials():
    return [dict(trial=i, seed=100+i, status='settled', pose_key='obsolete_group', group_id=99,
                 reference_pose_id=99, final_quat_xyzw=Rotation.from_euler('x', i, degrees=True).as_quat().tolist(),
                 final_pos_chute_mm=[480, 10, 10]) for i in range(2)]


def test_reconstruction_uses_first_trial_preserves_all_references_and_original_fields(tmp_path):
    mesh=tmp_path/'Part.stl';mesh.write_bytes(b'checksum fixture')
    raw=trials();before=json.dumps(raw)
    data, annotated=observations_from_trials(mesh, list(reversed(raw)), descriptor(mesh))
    assert len(data['poses'])==1
    pose=data['poses'][0]
    assert pose['quaternion_xyzw']==raw[0]['final_quat_xyzw']
    assert pose['source_trial']==0 and pose['trial_ids']==[0,1]
    assert {tuple(q) for q in pose['reference_quaternions_xyzw']}=={tuple(r['final_quat_xyzw']) for r in raw}
    assert json.dumps(raw)==before
    assert all(all(row[k]==original[k] for k in original) for row,original in zip(annotated,raw))
    assert {r['source_pose_id'] for r in annotated}=={pose['id']}


def test_compatible_anchor_is_kept_instead_of_first_trial_and_mismatched_mesh_is_rejected(tmp_path):
    mesh=tmp_path/'Part.stl';mesh.write_bytes(b'checksum fixture')
    recognition=descriptor(mesh);anchor=Rotation.from_euler('x',.4,degrees=True).as_quat().tolist()
    baseline=dict(schema_version=1,geometry={'stl_sha256':hashlib.sha256(mesh.read_bytes()).hexdigest()},
                  recognition=recognition,poses=[dict(id='Part_0042',quaternion_xyzw=anchor,position_com_chute_mm=[480,10,10])])
    data, rows=observations_from_trials(mesh,trials(),recognition,baseline=baseline)
    assert data['poses'][0]['id']=='Part_0042'
    assert data['poses'][0]['quaternion_xyzw']==anchor
    assert len(data['poses'][0]['reference_quaternions_xyzw'])==3
    assert {r['source_pose_id'] for r in rows}=={'Part_0042'}
    baseline['geometry']['stl_sha256']='wrong'
    with pytest.raises(ValueError,match='SHA-256'):
        observations_from_trials(mesh,trials(),recognition,baseline=baseline)


@pytest.mark.parametrize('continuous',[False,True])
def test_reconstruction_merges_symmetry_equivalent_orientations(tmp_path,continuous):
    mesh=tmp_path/'Part.stl';mesh.write_bytes(b'checksum fixture')
    recognition=descriptor(mesh)
    if continuous:recognition.update(continuous_axis_part=[0,0,1],symmetry_symbol='Cinf')
    else:recognition.update(symmetry_quaternions_xyzw=Rotation.from_euler('z',[[0],[120],[240]],degrees=True).as_quat().tolist(),symmetry_symbol='C3')
    rows=trials()
    for row,angle in zip(rows,[0,120]):row['final_quat_xyzw']=Rotation.from_euler('z',angle,degrees=True).as_quat().tolist()
    data,_=observations_from_trials(mesh,rows,recognition)
    assert len(data['poses'])==1 and data['poses'][0]['count']==2


def test_upstream_json_alias_and_explicit_conflicts(tmp_path):
    mesh=tmp_path/'Part.stl';mesh.write_bytes(b'fixture')
    config=RunConfig.from_dict(dict(mesh_path=str(mesh),grouping_enabled=True,compute_stability=True))
    assert config.group_poses and config.grouping_enabled
    assert .5 in config.disturbance_levels_mm
    with pytest.raises(ValueError,match='Conflicting'):
        RunConfig.from_dict(dict(mesh_path=str(mesh),grouping_enabled=True,group_poses=False))


def test_exact_published_dl4a_observed_memberships_and_frequency_cutoff():
    fixture=Path(__file__).resolve().parents[1]/'experiments/grouped_pose_study/baseline/Dl4a_observed_fixture.json'
    data=json.loads(fixture.read_text())
    result=analyze_groups(catalog_models()['Dl4a'],data['poses'],data['recognition'])
    assert len(data['poses'])==29 and result['groups']==data['expected_groups']
    counts={p['id']:p['count'] for p in data['poses']}
    assert len(result['groups'])==20
    assert sum(sum(counts[k] for k in group)*100/data['completed_trials']>=1 for group in result['groups'])==6


def test_partial_assessment_resumes_truncated_tail_without_repeating_drops(tmp_path,monkeypatch):
    import trimesh
    from dashas_drop_sim.physics import ChuteSimulator
    from dashas_drop_sim.runner import run_experiment
    mesh=tmp_path/'Cube.stl';trimesh.creation.box(extents=[20,20,20]).export(mesh)
    config=RunConfig(mesh,output_dir=tmp_path/'runs',trials=1,workers=1,drop_height_mm=0,length_mm=500,
                     grouping_barrier_mm=.005,disturbance_levels_mm=(0,.005))
    stopped=False
    def emit(event):
        nonlocal stopped
        if event.get('event')=='stability_progress' and event['completed']==3:stopped=True
    first=run_experiment(config,emit=emit,cancel=lambda:stopped)
    assert not first.summary['group_assessment_complete'] and first.summary['group_disturbance_trials']==3
    raw=(first.run_dir/'trials.jsonl').read_bytes()
    progress=first.run_dir/'group_disturbances.jsonl'
    prefix=progress.read_bytes()
    with progress.open('ab') as f:f.write(b'{"disturbance_index":')
    monkeypatch.setattr(ChuteSimulator,'drop',lambda *_a,**_kw:pytest.fail('Saved drops must not be rerun'))
    resumed=run_experiment(config,resume_run_dir=first.run_dir)
    assert resumed.summary['group_assessment_complete'] and resumed.summary['group_disturbance_trials']==13
    assert progress.read_bytes().startswith(prefix)
    records=[json.loads(line) for line in progress.read_text().splitlines()]
    assert len({r['disturbance_index'] for r in records})==13
    assert (first.run_dir/'trials.jsonl').read_bytes()==raw
    config.mu_wall=.2
    with pytest.raises(ValueError,match='configuration'):
        run_experiment(config,resume_run_dir=first.run_dir)


def test_ambiguous_pose_catalogue_reference_does_not_duplicate_observed_frequency(tmp_path,monkeypatch):
    import trimesh
    from dashas_drop_sim.pose_matching import PoseResolver,PoseMatch
    from dashas_drop_sim.runner import run_experiment
    mesh=tmp_path/'Cube.stl';trimesh.creation.box(extents=[20,20,20]).export(mesh)
    config=RunConfig(mesh,output_dir=tmp_path/'runs',trials=1,workers=1,drop_height_mm=0,length_mm=500,
                     grouping_barrier_mm=.005,disturbance_levels_mm=(0,.005))
    original=PoseResolver.resolve
    def reference_ambiguity(self,quat):
        return PoseMatch('ambiguous',None,.1) if type(self) is PoseResolver else original(self,quat)
    monkeypatch.setattr(PoseResolver,'resolve',reference_ambiguity)
    result=run_experiment(config)
    frequencies=result.summary['pose_frequencies']
    assert sum(r['count'] for r in frequencies)==1
    assert len(frequencies)==1 and frequencies[0]['pose_id']=='group_000'
    raw=json.loads((result.run_dir/'trials.jsonl').read_text())
    assert raw['match_status']=='ambiguous'


def test_preview_uses_supplied_observed_catalogue_ids(tmp_path):
    import threading
    import trimesh
    from dashas_drop_sim.cli import _preview
    from dashas_drop_sim.runner import run_experiment
    mesh=tmp_path/'Cube.stl';trimesh.creation.box(extents=[20,20,20]).export(mesh)
    config=RunConfig(mesh,output_dir=tmp_path/'runs',trials=1,workers=1,drop_height_mm=0,length_mm=500,
                     group_poses=False,compute_stability=False)
    initial=run_experiment(config)
    raw=json.loads((initial.run_dir/'trials.jsonl').read_text())
    manifest=json.loads((initial.run_dir/'manifest.json').read_text())
    catalogue=tmp_path/'poses.json'
    catalogue.write_text(json.dumps(dict(schema_version=1,geometry={'stl_sha256':manifest['mesh_source_sha256']},
        recognition=manifest['recognition'],poses=[dict(id='Cube_anchor',quaternion_xyzw=raw['final_quat_xyzw'])])),encoding='utf-8')
    config.observed_catalog_path=catalogue
    before=set(config.output_dir.glob('run_*'))
    assert _preview(config,0,visible=False,stopped=threading.Event())==0
    new=set(config.output_dir.glob('run_*'))-before
    assert len(new)==1
    preview=json.loads((new.pop()/'preview.json').read_text())
    assert preview['match_status']=='matched' and preview['reference_pose_id']=='Cube_anchor'
