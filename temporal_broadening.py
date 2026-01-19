#!/usr/bin/env python3
"""
Minimal plot of emitted vs received waveform from a HELIOS++ fullwave file.

Quick edit defaults (no CLI needed):
  FULLWAVE_PATH = Path("output/.../leg000_fullwave.txt")
  ROW_IDX = 0
  PULSE_LENGTH_NS = 4.0
  TITLE = "Emitted vs Received Waveform"

CLI still works and overrides defaults:
  python temporal_broadening.py --fullwave path/to/leg000_fullwave.txt --row 0 --pulse-length-ns 4.0 --title "My title"
"""
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# -------- Easy-to-edit defaults -------- #
FULLWAVE_PATH = Path(
    "output/Survey4_Cube_waveform_low/4_Cube_half_area_of_footprint_centered/2026-01-14_10-43-53/leg000_fullwave.txt"
)
ROW_IDX = 0
PULSE_LENGTH_NS = 4.0
TITLE = "Emitted vs Received Waveform"
# --------------------------------------- #


def plot_waveform(fullwave: Path, row_idx: int, pulse_length_ns: float, title: str) -> Path:
    with fullwave.open("r") as f:
        try:
            line = next(line for i, line in enumerate(f) if i == row_idx)
        except StopIteration:
            raise IndexError(f"Row {row_idx} out of range for {fullwave}")
    row = np.array([float(x) for x in line.split()])

    received = row[10:]
    t_received = np.linspace(row[7], row[8], received.size)
    rel_t = t_received - t_received.min()

    bin_size = rel_t[1] - rel_t[0] if received.size > 1 else pulse_length_ns / 10.0
    t_emit = np.arange(received.size) * bin_size
    tau = (pulse_length_ns * 0.5) / 3.5
    emitted = (t_emit / tau) ** 2 * np.exp(-t_emit / tau)
    emitted *= received.max() / emitted.max()

    plt.figure(figsize=(8, 4))
    plt.plot(t_emit, emitted, label="Emitted waveform", lw=2)
    plt.plot(rel_t, received, label="Received waveform", alpha=0.85)
    plt.xlabel("Time [ns] (relative)")
    plt.ylabel("Energy / Signal")
    plt.title(title)
    plt.legend()
    plt.grid(alpha=0.15)
    plt.tight_layout()

    out_path = fullwave.with_name(f"leg{row_idx:03d}_emitted_vs_received.png")
    plt.savefig(out_path, dpi=150)
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot emitted vs received waveform.")
    parser.add_argument("--fullwave", type=Path, default=FULLWAVE_PATH, help="Path to legXXX_fullwave.txt")
    parser.add_argument("--row", type=int, default=ROW_IDX, help="Zero-based row index to plot")
    parser.add_argument("--pulse-length-ns", type=float, default=PULSE_LENGTH_NS, help="Pulse length for emitted model")
    parser.add_argument("--title", type=str, default=TITLE, help="Plot title")
    args = parser.parse_args()

    out_path = plot_waveform(args.fullwave, args.row, args.pulse_length_ns, args.title)
    print(f"Saved plot to: {out_path}")


if __name__ == "__main__":
    main()
