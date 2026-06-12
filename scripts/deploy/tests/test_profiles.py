from pathlib import Path

import pytest
import yaml
from deploy.profiles import ProfileError, ProfileStore


def write_profiles(directory: Path, profiles: dict) -> None:
    (directory / "known_targets.yaml").write_text(
        yaml.safe_dump(
            {
                "172.20.1.11": {
                    "hostname": "nuc1",
                    "robot_name": "amy",
                }
            }
        )
    )
    (directory / "profiles.yaml").write_text(yaml.safe_dump(profiles))


def test_load_and_resolve_manual_target(tmp_path: Path) -> None:
    write_profiles(
        tmp_path,
        {
            "balls": {"kid": {"diameter": 0.14}},
            "matches": {
                "lab": {
                    "field": "labor",
                    "ball": "kid",
                    "wifi_profile": "Lab",
                    "game_settings": {
                        "team_id": 6,
                        "team_color": 0,
                        "role": "offense",
                        "position_number": 0,
                        "monitoring_host_ip": "172.20.0.10",
                    },
                }
            },
        },
    )

    store = ProfileStore.load(tmp_path)
    targets = store.resolve_targets(
        ["amy", "robot.example"],
        user="robot",
        workspace="~/ws",
    )

    assert targets[0].name == "nuc1"
    assert targets[0].destination == "robot@172.20.1.11"
    assert targets[1].host == "robot.example"


def test_rejects_invalid_ball_profile(tmp_path: Path) -> None:
    write_profiles(
        tmp_path,
        {
            "balls": {"kid": {"diameter": -1}},
            "matches": {},
        },
    )

    with pytest.raises(ProfileError, match="positive"):
        ProfileStore.load(tmp_path)
