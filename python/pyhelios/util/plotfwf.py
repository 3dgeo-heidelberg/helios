import matplotlib.pyplot as plt
import numpy as np
import sys
from pathlib import Path

c_in_ns = 0.299792458

file = Path(r"output\Survey Playback\RAMI")
file = sorted(file.iterdir())[-1]
file = file / "points" / "leg000_fullwave.txt"

with open(file, "r") as f:
    data = f.readline()
data = list(map(float, data.split(" ")))

tmin = data[7]
tmax = data[8]
fwf = data[10:]

bins = (np.linspace(tmin, tmax, len(fwf))) * c_in_ns

print("%.0f bins, %.3f ns per bin" % (len(bins), (tmax - tmin) / len(bins)))

fig = plt.figure(figsize=(10, 3))
plt.plot(bins, fwf)
# plt.xlim([5, 8])
plt.tight_layout()
plt.savefig(str(file) + ".png")
