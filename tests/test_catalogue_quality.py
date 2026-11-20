import copy
import json
from pathlib import Path
import pytest
import trimesh
from dashas_drop_sim.catalogue_quality import annotate_repository, write
from dashas_drop_sim.pose_quality import sha256, QualityCancelled

def catalogue_repo(tmp_path):
    repo=tmp_path/'repo'
    (repo/'.git').mkdir(parents=True)
    (repo/'README.md').write_text('# Catalogues\n')
    workpieces=[]
    for index in range(39):
        name=f'T{index}'
        part=repo/name
        (part/'run').mkdir(parents=True)
        mesh=part/f'{name}.stl'
        trimesh.creation.box(extents=(10,20,30)).export(mesh)
        write(part/'run/config.json',{'alpha_deg':45,'beta_deg':0})
        folder=part/'catalogue/grouped_10000/moving'
        folder.mkdir(parents=True)
        write(folder/'grouping-evidence.json',{'model':'observed-seated-geodesic-basins-v1','sampling_step_deg':.1,'paths':[]})
        data={'schema_version':1,'workpiece':name,'condition':'moving','total_trials':10000,'settled_trials':10000,
            'settings':{'recognition':{'symmetry_quaternions_xyzw':[[0,0,0,1]]},'transverse_angle_deg':45.,'longitudinal_angle_deg':0.},
            'source':{'mesh_path':f'{name}/{name}.stl','mesh_sha256':sha256(mesh),'run_dir':f'{name}/run','config_sha256':sha256(part/'run/config.json')},
            'poses':[{'id':f'{name}_0001','count':10000,'incidence_percent':100.,'quaternion_xyzw':[0,0,0,1],'group_id':'G0'}],
            'groups':[{'id':'G0','members':[f'{name}_0001']}],'grouping_evidence':'grouping-evidence.json',
            'grouping_evidence_sha256':sha256(folder/'grouping-evidence.json')}
        write(folder/'catalogue.json',data)
        write(folder/'files.sha256.json',{p.name:sha256(p) for p in folder.iterdir() if p.is_file()})
        (part/'README.md').write_text(f'# {name}\n\n## Moving belt\n\n| Pose | Group | Preview | Pose drops | Pose incidence | Group drops | Group incidence |\n| --- | --- | --- | ---: | ---: | ---: | ---: |\n| P000<br><code>{name}_0001</code> | G0 | image | 10000 | 100% | 10000 | 100% |\n')
        workpieces.append({'workpiece':name,'grouped_10000_catalogues':{'moving':{'path':(folder/'catalogue.json').relative_to(repo).as_posix()}}})
    write(repo/'index.json',{'schema_version':1,'workpieces':workpieces})
    return repo

def payloads(repo):
    return {p.relative_to(repo).as_posix():p.read_bytes() for p in repo.rglob('*') if p.is_file() and '.git' not in p.relative_to(repo).parts}

def test_cancel_leaves_every_catalogue_unchanged(tmp_path):
    repo=catalogue_repo(tmp_path)
    before=payloads(repo)
    def progress(message):
        raise QualityCancelled('cancelled')
    with pytest.raises(QualityCancelled):
        annotate_repository(repo,condition='moving',progress=progress)
    assert payloads(repo)==before

def test_failure_mid_publication_rolls_back_all_catalogues(tmp_path,monkeypatch):
    import dashas_drop_sim.catalogue_quality as module
    repo=catalogue_repo(tmp_path)
    before=payloads(repo)
    original=module.os.replace
    calls=0
    def fail_second(*args):
        nonlocal calls
        calls+=1
        if calls==2:
            raise OSError('injected publication failure')
        return original(*args)
    monkeypatch.setattr(module.os,'replace',fail_second)
    with pytest.raises(OSError,match='injected'):
        annotate_repository(repo,condition='moving')
    assert payloads(repo)==before

def test_repeated_annotation_preserves_data_and_single_stability_column(tmp_path):
    repo=catalogue_repo(tmp_path)
    annotate_repository(repo,condition='moving')
    first=payloads(repo)
    annotate_repository(repo,condition='moving')
    assert payloads(repo)==first
    assert (repo/'T0/README.md').read_text().count(' Stability |')==1
