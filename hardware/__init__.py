"""Default vehicle parameters shared by simulation and control."""

import json
from pathlib import Path


_profile = json.loads(Path(__file__).with_name("control_profile.json").read_text(encoding="utf-8"))
PARAMETERS = {
    section: {name: entry["value"] for name, entry in fields.items()}
    for section, fields in _profile.items()
}
