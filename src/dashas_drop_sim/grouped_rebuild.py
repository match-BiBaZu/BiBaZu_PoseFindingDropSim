"""Rebuild and assess published pose basins from immutable saved drop records."""
from collections import Counter
import hashlib
import json
from pathlib import Path

from .config import RunConfig
from .geometry import prepare_part
from .physics import ChuteSimulator
from .observed import ObservedPoseResolver, observations_from_trials
from .grouped_run import assess_and_export, SOURCE_REVISION
from .runner import _write_json, build_manifest


def rebuild_saved_run(source, output, *, mesh_path=None, cache_dir=None, emit=None, cancel=None):
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output or output.is_relative_to(source):
        raise ValueError("Assessment output must be separate from the original run.")
    emit, cancel = emit or (lambda _e: None), cancel or (lambda: False)
    raw_bytes = (source / "trials.jsonl").read_bytes()
    raw = [json.loads(line) for line in raw_bytes.splitlines()]
    original = json.loads((source / "manifest.json").read_text(encoding="utf-8"))
    config_data = json.loads((source / "config.json").read_text(encoding="utf-8"))
    if len(raw) != len({r["trial"] for r in raw}) or len(raw) != len({r["seed"] for r in raw}):
        raise ValueError("Saved run contains duplicate trial IDs or seeds.")
    config_data = {k:v for k,v in config_data.items() if k in RunConfig.__dataclass_fields__}
    original_cache = Path(config_data["output_dir"]) / ".mesh_cache"
    config_data.update(mesh_path=str(mesh_path or config_data["mesh_path"]), output_dir=str(output),
                       group_poses=True, compute_stability=True, catalog_repo=None, catalog_push=False,
                       workers=4, disturbance_levels_mm=[0, .05, .1, .2, .4, .5, .8])
    config = RunConfig.from_dict(config_data)
    mesh_hash = hashlib.sha256(config.mesh_path.read_bytes()).hexdigest()
    if mesh_hash != original["mesh_source_sha256"]:
        raise ValueError("Selected STL differs from the saved drop run.")
    if len(raw) != config.trials:
        raise ValueError("Saved drop run is incomplete.")
    output.mkdir(parents=True, exist_ok=True)
    for name in ("trials.jsonl", "trials.csv", "config.json", "manifest.json", "summary.json"):
        src = source / name
        if not src.exists():
            continue
        target = output / "source" / name
        target.parent.mkdir(exist_ok=True)
        if target.exists() and target.read_bytes() != src.read_bytes():
            raise ValueError(f"Saved assessment input changed: {name}")
        if not target.exists():
            target.write_bytes(src.read_bytes())
    baseline = (json.loads(config.observed_catalog_path.read_text(encoding="utf-8-sig"))
                if config.observed_catalog_path else None)
    observations, annotated = observations_from_trials(config.mesh_path, raw, original["recognition"], baseline=baseline)
    resolver = ObservedPoseResolver(config.mesh_path, data=observations)
    part = prepare_part(config.mesh_path, Path(cache_dir) if cache_dir else original_cache, config.density_g_cm3)
    if part.source_sha256 != mesh_hash:
        raise ValueError("Prepared collision model has a different source mesh.")
    simulator = ChuteSimulator(config, part)
    _write_json(output / "config.json", config.to_dict())
    _write_json(output / "original_observations.json", observations)
    manifest = build_manifest(config, part, resolver)
    manifest.update(grouping_source_revision=SOURCE_REVISION,
                    grouped_catalog_revision="632b830bd890498e1b07b14d44d97b25f5bba7bb",
                    source_run=str(source), source_trials_sha256=hashlib.sha256(raw_bytes).hexdigest(),
                    drop_trials_reused=len(raw), drop_trials_executed=0,
                    source_status_counts=dict(Counter(r["status"] for r in raw)))
    _write_json(output / "manifest.json", manifest)
    summary = assess_and_export(config, simulator, resolver, annotated, output, cancel, emit)
    summary.update(completed_drop_trials=len(raw), source_status_counts=manifest["source_status_counts"],
                   source_trials_sha256=manifest["source_trials_sha256"],
                   observed_source_poses=len(observations["poses"]), cancelled=bool(cancel()))
    _write_json(output / "summary.json", summary)
    if (source / "trials.jsonl").read_bytes() != raw_bytes:
        raise ValueError("Original drop records changed during assessment.")
    return summary
