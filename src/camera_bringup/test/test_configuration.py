from pathlib import Path
import pytest
import yaml
from camera_bringup.configuration import load_camera

ROOT = Path(__file__).parents[1] / "config"


def test_independent_profiles():
    wrist = load_camera(ROOT / "wrist.yaml", "wrist", "target")
    scene = load_camera(ROOT / "scene.yaml", "scene", "usb2_15fps")
    assert wrist["serial_no"] == "_934222072657"
    assert scene["serial_no"] == "_238222073675"
    assert wrist["rgb_camera.color_profile"] == "640,480,30"
    assert scene["depth_module.depth_profile"] == "640,480,15"
    assert scene["enable_sync"] and scene["align_depth.enable"]


def test_unknown_profile_has_no_fallback():
    with pytest.raises(ValueError):
        load_camera(ROOT / "scene.yaml", "scene", "automatic")


@pytest.mark.parametrize("change", [
    {"serial": 238222073675},
    {"extra": True},
    {"profiles": {"bad": {"color_profile": "640,480,30"}}},
])
def test_invalid_configuration(tmp_path, change):
    data = yaml.safe_load((ROOT / "scene.yaml").read_text())
    data.update(change)
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_camera(path, "scene", "bad" if "profiles" in change else "target")
