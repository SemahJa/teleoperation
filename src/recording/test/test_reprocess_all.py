import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

script=Path(__file__).resolve().parents[3]/'scripts/data_collection/reprocess_all_demonstrations.py'
spec=importlib.util.spec_from_file_location('cable_reprocess',script)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_historical_indices_come_from_recorded_identity_mapping(tmp_path):
    reference=tmp_path/'reference';p=reference/'provenance/sources';p.mkdir(parents=True)
    (p/'arbitrary_name.json').write_text(json.dumps(dict(source_id='known_original_id',source_number=17)))
    assert module.historical_mapping(reference)=={'known_original_id':17}
    (p/'conflicting.json').write_text(json.dumps(dict(source_id='known_original_id',source_number=2)))
    with pytest.raises(ValueError):module.historical_mapping(reference)


def test_outcome_sets_preserve_user_labels():
    assert module.LABELS['failure']==[0,15]
    assert module.LABELS['partial_success']==[20]
    assert module.LABELS['success_after_recovery']==[1,9,11,19]
    assert module.LABELS['direct_success']==[2,3,4,5,8,10,12,13,14,16,17,18,21]
    flat=[i for values in module.LABELS.values() for i in values]
    assert len(flat)==len(set(flat))
    assert 22 not in flat  # Additional historical episode stays unknown.


def test_one_bad_source_does_not_prevent_other_sources(tmp_path,monkeypatch):
    rows=[dict(episode_id=name,path=str(tmp_path/name),source_number=i,group='additional',outcome='unknown',success=None,inventory={},conversion_status='pending') for i,name in enumerate(['corrupt','idle'])]
    monkeypatch.setattr(module,'discover',lambda *a:(rows,{}));calls=[]
    def prepare(bag,number,work,settings):
        calls.append(bag.name)
        if bag.name=='corrupt':raise ValueError('synthetic unreadable source')
        path=work/'empty.npz';np.savez_compressed(path)
        return dict(npz=str(path),summary={'retained_frames':0},supervision=dict(native_gripper_runs=[],native_arm_enabled_samples=0,native_gripper_busy_samples=0),native_evidence={})
    monkeypatch.setattr(module.core,'prepare',prepare)
    monkeypatch.setattr(module,'aggregate_group',lambda *a:None)
    args=SimpleNamespace(output=tmp_path/'out',resume=False,roots=[],historical_reference=tmp_path,cache_roots=[])
    assert module.run(args)==2
    result=json.loads((args.output/'inventory.json').read_text())
    assert calls==['corrupt','idle']
    assert [r['conversion_status'] for r in result]==['failed','idle']


def test_corrupt_metadata_is_inventoried_without_stopping_discovery(tmp_path,monkeypatch):
    import yaml
    monkeypatch.setattr(module,'historical_mapping',lambda p:{})
    for name in ['quest_bad','quest_valid']:
        d=tmp_path/name;d.mkdir();(d/'x.mcap').write_bytes(b'fixture')
        (d/'episode_manifest.json').write_text('{}')
        (d/'metadata.yaml').write_text('[' if name=='quest_bad' else yaml.safe_dump({'rosbag2_bagfile_information':{'starting_time':{'nanoseconds_since_epoch':1793355212533006786},'duration':{'nanoseconds':1000000000}}}))
    rows,_=module.discover([tmp_path],tmp_path/'reference')
    assert len(rows)==2
    byid={r['episode_id']:r for r in rows}
    assert byid['quest_bad']['conversion_status']=='failed'
    assert 'Unreadable recording metadata' in byid['quest_bad']['error']
    assert byid['quest_valid']['conversion_status']=='pending'


def test_audit_writes_complete_inventory_csv(tmp_path):
    import csv
    audit_script=Path(__file__).resolve().parents[3]/'scripts/data_collection/audit_reprocessed_datasets.py'
    audit_spec=importlib.util.spec_from_file_location('cable_audit',audit_script)
    audit_mod=importlib.util.module_from_spec(audit_spec);audit_spec.loader.exec_module(audit_mod)
    bundle=tmp_path/'bundle';bundle.mkdir()
    trainer=tmp_path/'trainer'
    (trainer/'src/robot_policy_learning').mkdir(parents=True)
    (trainer/'configs').mkdir(parents=True)
    (trainer/'src/robot_policy_learning/data.py').write_text('Catalog=Windows=normalization=window_starts=None;IMAGE="img";STATE="state"\n')
    (trainer/'configs/baseline.json').write_text('{}\n')
    (bundle/'inventory.json').write_text(json.dumps([dict(
        source_number=6,episode_id='quest_idle',recorded_start_utc='2026-10-28T16:23:08Z',
        group='historical',outcome='unknown',success=None,conversion_status='idle',
        original_duration_s=39.5,supervision={'samples_30hz':{'opening':0,'closing':0,'neutral':0,'invalid':0},'unrepresented_gripper_runs':0}
    )]))
    audit_mod.audit(bundle,trainer,partial=False)
    with (bundle/'inventory.csv').open() as f:
        rows=list(csv.DictReader(f))
    assert len(rows)==1
    assert rows[0]['episode_id']=='quest_idle'
    assert rows[0]['legacy_non_neutral_gripper_frames']=='0'
