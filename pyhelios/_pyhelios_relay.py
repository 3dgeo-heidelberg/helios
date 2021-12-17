import sys
from pathlib import Path

# append run folder to sys path before import
sys.path.append(str(Path(__file__).parent.parent / "run"))
from _pyhelios import *