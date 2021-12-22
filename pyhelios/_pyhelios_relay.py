import sys
from pathlib import Path
try:
    import osgeo  # if osgeo/gdal is installed, import it _before_ importing pyhelios or altering the PATH
except:
    pass
# append run folder to sys path before import
sys.path.append(str(Path(__file__).parent.parent / "run"))
from _pyhelios import *