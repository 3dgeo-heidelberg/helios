import sys
from pathlib import Path

helios_root = str(Path(__file__).parent.parent.absolute())
sys.path.append(helios_root)
import pyhelios

print(pyhelios.getVersion())
sim = pyhelios.Simulation()
print(
    'Simulation started? {status}'
    .format(status="YES" if sim.isStarted() else "NO")
)
