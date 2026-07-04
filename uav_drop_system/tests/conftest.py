"""
uav_drop_system/ dizinini sys.path'e ekler.

Proje içindeki modüller birbirini `import config`, `from vision.detector
import ...` gibi düz (flat) isimlerle import eder (old_docs'taki tek dosyalık
prototipe yakın bir stil, Raspberry Pi üzerinde `cd uav_drop_system && python
main.py` ile doğrudan çalıştırmayı kolaylaştırır). pytest testlerinin de aynı
isimlerle import edebilmesi için bu conftest, testler toplanmadan önce
uav_drop_system/ dizinini sys.path'in başına ekler.
"""

import os
import sys

_UAV_DROP_SYSTEM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

if _UAV_DROP_SYSTEM_DIR not in sys.path:
    sys.path.insert(0, _UAV_DROP_SYSTEM_DIR)
