import json
from pathlib import Path

import pytest

from dashas_drop_sim.catalog import catalog_models
from dashas_drop_sim.grouping import analyze_groups
from dashas_drop_sim.rolling_basins import energy_basins, scoped_model


def solve(heights, barrier=.5, observations=None):
    return energy_basins(heights, list(zip(range(len(heights) - 1), range(1, len(heights)))),
                         observations or {str(i): i for i in range(len(heights))}, barrier)


def test_flat_chain_has_no_whole_group_diameter_guard():
    assert solve([0] * 8)[0] == [[str(i) for i in range(8)]]


def test_separating_saddle_and_ambiguous_high_observation():
    groups, review, minima, _ = solve([0, 2, 0])
    assert groups == [["0"], ["1"], ["2"]]
    assert review[0]["members"] == ["1"]
    assert len(review[0]["destinations"]) == 2
    assert len(minima) == 2


def test_gradual_steps_do_not_bridge_a_true_barrier():
    heights = [0, .2, .4, .6, .8, 1, .8, .6, .4, .2, 0]
    groups, _, minima, _ = solve(heights)
    assert len(minima) == 2
    assert not any("0" in group and "10" in group for group in groups)


def test_downhill_asymmetric_rolling_family_stays_together():
    assert solve([0, .4, .8, 1.2, 1.6, 2])[0] == [[str(i) for i in range(6)]]


@pytest.mark.parametrize("saddle,expected", [(.5, 1), (.5 + 2e-9, 2), (0, 1)])
def test_escape_barrier_boundary(saddle, expected):
    _, _, minima, _ = solve([0, saddle, 0])
    assert len(minima) == expected


def test_input_order_invariant():
    heights = [0, .1, .2, 2, .1, 0]
    edges = list(zip(range(5), range(1, 6)))
    obs = {str(i): i for i in range(6)}
    a = energy_basins(heights, edges, obs, .5)
    b = energy_basins(heights, list(reversed(edges)), dict(reversed(list(obs.items()))), .5)
    assert a == b


def test_scope_requires_both_identity_and_hash(tmp_path):
    model = catalog_models()["Kl4i"]
    assert scoped_model(model) == "Kl4i"
    renamed = tmp_path / "other.stl"
    renamed.write_bytes(model.read_bytes())
    assert scoped_model(renamed) is None
    changed = tmp_path / "Kl4i.stl"
    changed.write_bytes(b"changed")
    assert scoped_model(changed) is None


def test_full_saved_kl4i_has_four_basins_and_preserves_order():
    fixture = json.loads((Path(__file__).parent / "fixtures/rolling_Kl4i.json").read_text())
    a = analyze_groups(catalog_models()["Kl4i"], fixture["members"], fixture["recognition"])
    b = analyze_groups(catalog_models()["Kl4i"], list(reversed(fixture["members"])), fixture["recognition"])
    assert a == b
    assert len(a["groups"]) == 4
    assert sorted(map(len, a["groups"])) == [2, 6, 16, 17]
    assert not a["review"]
    owner = {key: i for i, group in enumerate(a["groups"]) for key in group}
    assert any(path["angle_deg"] > 60 and owner[path["source"]] == owner[path["target"]]
               for path in a["paths"])
    assert all(path["angle_deg"] <= 60 + 1e-9 for path in a["paths"] if "sample_nodes" in path)
    assert sorted(m for group in a["groups"] for m in group) == sorted(m["id"] for m in fixture["members"])
