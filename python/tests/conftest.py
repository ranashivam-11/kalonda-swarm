import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))


def _sil_available() -> bool:
    try:
        from kalonda._paths import sil_library

        sil_library()
        return True
    except FileNotFoundError:
        return False


requires_sil = pytest.mark.skipif(not _sil_available(), reason="firmware SIL library not built (make firmware)")
