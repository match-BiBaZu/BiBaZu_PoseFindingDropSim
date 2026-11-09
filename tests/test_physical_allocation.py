"""Physical annotations query saved groups without changing or resimulating them."""
from copy import deepcopy
import json
from pathlib import Path

import pytest
from scipy.spatial.transform import Rotation

from dashas_drop_sim.catalog import catalog_models
from dashas_drop_sim import physical_allocation as allocation

IDENTITY = [0, 0, 0, 1]


@pytest.mark.parametrize('mode', ['running', 'stopped'])
def test_missing_qk1a_labels_join_existing_basins_without_drops(mode, monkeypatch):
    from dashas_drop_sim.physics import ChuteSimulator
    def forbidden(*_a, **_kw):
        raise AssertionError('Annotation must not create or run a simulator')
    monkeypatch.setattr(ChuteSimulator, '__init__', forbidden)
    monkeypatch.setattr(ChuteSimulator, 'drop', forbidden)
    fixture = json.loads((Path(__file__).parent / 'fixtures/physical_label_qk1a.json').read_text())[mode]
    before = deepcopy(fixture)
    result = allocation.allocate_physical_poses(catalog_models()['Qk1a'], fixture['members'],
                fixture['groups'], fixture['recognition'], [fixture['reference']])
    record = result['allocations'][0]
    assert record['status'] == 'assigned' and record['method'] == 'seated_basin'
    assert record['group_id'] == fixture['expected_group']
    assert record['partition_unchanged'] and record['recognition_distance_deg'] > 5
    assert result['drop_trials_executed'] == result['disturbance_trials_executed'] == 0
    assert fixture == before


def inputs(split=False):
    members = [dict(id=str(i), quaternion_xyzw=Rotation.from_euler('x', angle, degrees=True).as_quat().tolist())
               for i, angle in enumerate([0, 6])]
    groups = [dict(group_id=0, members=['0']), dict(group_id=1, members=['1'])] if split else [dict(group_id=0, members=['0', '1'])]
    query = [dict(physical_pose_label=4, quaternion_xyzw=Rotation.from_euler('x', 3, degrees=True).as_quat().tolist())]
    return members, groups, query


def test_ambiguity_inside_one_group_is_resolved_at_group_level(monkeypatch):
    members, groups, query = inputs()
    monkeypatch.setattr(allocation, 'analyze_groups', lambda *_a, **_kw: pytest.fail('Recognition must suffice'))
    record = allocation.allocate_physical_poses(Path('unused.stl'), members, groups, {}, query)['allocations'][0]
    assert (record['status'], record['group_id'], record['method']) == ('assigned', 0, 'observed_orientation')


def test_cross_group_ambiguity_stays_explicit(monkeypatch):
    members, groups, query = inputs(split=True)
    monkeypatch.setattr(allocation, 'analyze_groups', lambda *_a, **_kw: pytest.fail('Do not override ambiguous recognition'))
    record = allocation.allocate_physical_poses(Path('unused.stl'), members, groups, {}, query)['allocations'][0]
    assert record['status'] == 'ambiguous' and record['group_id'] is None


@pytest.mark.parametrize('continuous', [False, True])
def test_symmetry_and_quaternion_sign_are_respected(continuous):
    half = Rotation.from_euler('z', 180, degrees=True).as_quat().tolist()
    descriptor = {'symmetry_quaternions_xyzw': [IDENTITY, half]}
    if continuous:
        descriptor['continuous_axis_part'] = [0, 0, 1]
    q = (-Rotation.from_euler('z', 73 if continuous else 180, degrees=True).as_quat()).tolist()
    result = allocation.allocate_physical_poses(Path('unused.stl'), [dict(id='a', quaternion_xyzw=IDENTITY)],
                [dict(group_id=0, members=['a'])], descriptor, [dict(physical_pose_label=0, quaternion_xyzw=q)])
    assert result['allocations'][0]['group_id'] == 0


def test_reference_cannot_bridge_saved_groups(monkeypatch):
    members, groups, _ = inputs(split=True)
    query = [dict(physical_pose_label=0, quaternion_xyzw=Rotation.from_euler('z', 45, degrees=True).as_quat().tolist())]
    monkeypatch.setattr(allocation, 'analyze_groups', lambda *_a, **_kw: dict(groups=[['0', '1', 'physical_reference_query']], paths=[]))
    record = allocation.allocate_physical_poses(Path('unused.stl'), members, groups, {}, query)['allocations'][0]
    assert record['status'] == 'ambiguous' and record['group_id'] is None
    assert not record['partition_unchanged']


def test_distant_reference_is_not_assigned_to_nearest_group(monkeypatch):
    members, groups, _ = inputs()
    query = [dict(physical_pose_label=0, quaternion_xyzw=Rotation.from_euler('z', 90, degrees=True).as_quat().tolist())]
    monkeypatch.setattr(allocation, 'analyze_groups', lambda *_a, **_kw: dict(groups=[['0', '1'], ['physical_reference_query']], paths=[]))
    record = allocation.allocate_physical_poses(Path('unused.stl'), members, groups, {}, query)['allocations'][0]
    assert record['status'] == 'unassigned' and record['group_id'] is None


def test_duplicate_membership_is_rejected():
    members, groups, query = inputs(split=True)
    groups[1]['members'].append('0')
    with pytest.raises(ValueError, match='multiple saved groups'):
        allocation.allocate_physical_poses(Path('unused.stl'), members, groups, {}, query)
