"""Current settings survive a fresh GUI start and reproduce the production runs."""
import json
import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt6.QtWidgets import QApplication

from dashas_drop_sim.cli import _config
from dashas_drop_sim.config import RunConfig
from dashas_drop_sim.gui import DropSimulationWindow
from dashas_drop_sim.settings import (
    GUI_INITIAL_PARAMETERS, PACKAGED_SETTINGS_PATH, PARAMETER_KEYS, load_defaults, save_defaults,
)

CONFIGS = Path(__file__).resolve().parents[1] / "configs"


def test_packaged_settings_cover_every_parameter_and_have_no_machine_paths():
    settings = load_defaults(PACKAGED_SETTINGS_PATH)
    assert set(settings["parameters"]) == PARAMETER_KEYS
    assert settings["inputs"] == {}


def test_fresh_gui_uses_the_complete_saved_parameter_snapshot(tmp_path):
    app = QApplication.instance() or QApplication([])
    window = DropSimulationWindow(settings_path=tmp_path / "absent.json")
    try:
        assert json.loads(json.dumps(window._parameter_values())) == GUI_INITIAL_PARAMETERS
    finally:
        window.close()
        del app


def test_partial_saved_settings_override_packaged_values_without_resetting_others(tmp_path):
    settings = tmp_path / "defaults.json"
    save_defaults(settings, {"trials": 13, "contact_damping_ratio": 0.7, "catalog_push": True,
                             "mu_belt": 0.4, "mu_wall": 0.2, "group_poses": False}, {})
    app = QApplication.instance() or QApplication([])
    window = DropSimulationWindow(settings_path=settings)
    try:
        assert window.trials_spin.value() == 13
        assert window.contact_damping_spin.value() == 0.7
        assert window.catalog_push_check.isChecked()
        assert window.compute_stability_check.isChecked()
        assert window.length_spin.value() == 500
        assert window.belt_stop_check.isChecked()
        values = window._parameter_values()
        assert (values["mu_belt"], values["mu_wall"], values["group_poses"]) == (0.4, 0.2, False)
    finally:
        window.close()
        del app


def test_new_friction_defaults_and_explicit_historical_configs(tmp_path):
    config = RunConfig(mesh_path=tmp_path / "part.stl", output_dir=tmp_path)
    config.mesh_path.write_bytes(b"fixture: configuration validation only")
    assert (config.mu_belt, config.mu_wall, config.group_poses) == (0.1, 0.05, True)
    assert (GUI_INITIAL_PARAMETERS["mu_belt"], GUI_INITIAL_PARAMETERS["mu_wall"],
            GUI_INITIAL_PARAMETERS["group_poses"]) == (0.1, 0.05, True)
    historical = config.to_dict()
    historical.update(mu_belt=0.4, mu_wall=0.2, group_poses=False)
    loaded = RunConfig.from_dict(historical)
    assert (loaded.mu_belt, loaded.mu_wall, loaded.group_poses) == (0.4, 0.2, False)


@pytest.mark.parametrize("name,part,trials,workers,push", [
    ("gui-current.json", "Dk1i", 100, 4, False),
    ("dk1i-1000.json", "Dk1i", 1000, 16, False),
    ("kf1i-10000.json", "Kf1i", 10000, 16, True),
])
def test_portable_profiles_preserve_all_physics_and_run_sizes(name, part, trials, workers, push):
    if name == "gui-current.json":
        config = _config(CONFIGS / name)
    else:
        # Historical source files remain unchanged; the new procedure requires assessment.
        with pytest.raises(ValueError, match="compute_stability"):
            _config(CONFIGS / name)
        payload = json.loads((CONFIGS / name).read_text())
        assert payload["compute_stability"] is False
        payload["compute_stability"] = True
        payload["catalog_repo"] = None
        payload["mesh_path"] = str((CONFIGS / payload["mesh_path"]).resolve())
        config = RunConfig.from_dict(payload)
    assert config.mesh_path.stem == part
    assert config.trials == trials and config.workers == workers
    assert config.catalog_push is push
    current = _config(CONFIGS / "gui-current.json").to_dict()
    for key in PARAMETER_KEYS - {"trials", "workers", "catalog_push", "friction_cone", "mu_belt", "mu_wall"}:
        assert config.to_dict()[key] == current[key], key
    assert config.friction_cone == ("elliptic" if name == "gui-current.json" else "pyramidal")
    assert config.contact_damping_ratio == 0.19
    assert (config.mu_belt, config.mu_wall) == ((0.1, 0.05) if name == "gui-current.json" else (0.4, 0.2))
    assert config.roughness_active_duration_s == 1
    assert config.belt_stop_enabled and config.belt_stop_ramp_s == 0.2
    assert config.belt_stop_observation_s == 2
