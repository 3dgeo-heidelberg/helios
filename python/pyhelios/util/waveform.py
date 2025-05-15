import numpy as np
import matplotlib.pyplot as plt

pulselength_ns = 2.0

num_bins = 1000

time_wave = np.zeros((num_bins,))
step = pulselength_ns * 2 / num_bins
tau = (pulselength_ns * 0.5) / 3.5
peakValue = 0
for i in range(num_bins):
    t = i * step
    t_tau = t / tau
    pt = t_tau**2 * np.exp(-t_tau)
    time_wave[i] = pt
    if pt > peakValue:
        peakValue = pt
        peakIndex = i

offsetfac = 4
offset_ns = pulselength_ns / offsetfac
max_t = peakIndex / num_bins * 2 * pulselength_ns
plt.plot([-pulselength_ns, 2 * pulselength_ns], [0, 0], "gray")
plt.plot([0, 0], [0, peakValue], "r--")
plt.plot(
    np.linspace(-max_t - offset_ns, 2 * pulselength_ns - max_t - offset_ns, num_bins),
    time_wave,
    "gray",
    label=f"Pulse offset by pulselength / {offsetfac}",
)
plt.plot(
    np.linspace(-max_t + offset_ns, 2 * pulselength_ns - max_t + offset_ns, num_bins),
    time_wave,
    "gray",
)
plt.plot(
    np.linspace(-max_t, 2 * pulselength_ns - max_t, num_bins),
    time_wave,
    "b-",
    label="Pulse",
)

winsize_offs = int(offset_ns / step)
fwf = np.zeros((num_bins + 2 * winsize_offs,))
fwf[:num_bins] = time_wave
fwf[winsize_offs : num_bins + winsize_offs] += time_wave
fwf[winsize_offs * 2 : num_bins + winsize_offs * 2] += time_wave
# fwf = fwf / np.max(fwf) * np.max(time_wave)
plt.plot(
    np.linspace(
        -max_t - offset_ns,
        2 * pulselength_ns - max_t + offset_ns,
        num_bins + 2 * winsize_offs,
    ),
    fwf,
    "k",
    label="Resulting full waveform (scaled)",
)


plt.ylabel("Energy/Signal Strength [DN]")
plt.xlabel("Time since maximum [ns]")
plt.annotate(
    text="",
    xy=(pulselength_ns / 2, peakValue),
    xytext=(-pulselength_ns / 2, peakValue),
    arrowprops=dict(arrowstyle="<->"),
)
plt.annotate("pulse length", xy=(0, peakValue + 0.01))
plt.annotate(
    text="",
    xy=(0, peakValue + 0.03),
    xytext=(pulselength_ns / 4, peakValue + 0.03),
    arrowprops=dict(arrowstyle="<->"),
)
plt.annotate(
    text="",
    xy=(0, peakValue + 0.03),
    xytext=(-pulselength_ns / 4, peakValue + 0.03),
    arrowprops=dict(arrowstyle="<->"),
)
plt.annotate("default win size", xy=(0, peakValue + 0.04))
plt.xlim(-pulselength_ns, 2 * pulselength_ns)
plt.xlim(-1, 5)
plt.ylim(-0.1, peakValue + 0.1)
plt.legend()
plt.show()
