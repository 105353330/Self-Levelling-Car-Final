# Creates a python constants document based off constants.h

import re
from pathlib import Path

_PATH = Path(__file__).resolve().parent / "constants.h"
_LINE = re.compile(r"constexpr\s+(?:unsigned\s+)?(?:int|long|float|double)\s+(\w+)\s*=\s*(-?\d+(?:\.\d+)?)f?\s*;")


def _parse(value):
    return float(value) if "." in value else int(value)


_values = {name: _parse(value) for name, value in _LINE.findall(_PATH.read_text())}
globals().update(_values)
