import json

import pytest

from src.mission_config import load_mission


@pytest.mark.parametrize("key,value", [("target_altitude", 0), ("safety_distance", -1),
                                       ("transit_flyby_progress", 1.5),
                                       ("use_route_shaping", "false"),
                                       ("required_checkpoints", [[1.1, 2], [3, 4], [5, 6]])])
def test_invalid_config(tmp_path, key, value):
    config = load_mission()
    config[key] = value
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        load_mission(path)
