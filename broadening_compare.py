#!/usr/bin/env python3
"""
Scan plane/plane_tilt/step/cube fullwave files (Cartesian and Polar), compute
broadening metrics, write CSVs, and plot the matched pulse (same fullwaveIndex)
with the maximum received FWHM across the matched set. Also plot mean emitted/
received waveforms (resampled to a common time grid) per dataset. Legends include
FWHM and effective width for the plotted pulse.
"""
from pathlib import Path
from typing import Dict, List, Tuple

import argparse
import matplotlib.pyplot as plt
import numpy as np


def emitted_wave(num_bins: int, bin_size: float, pulse_len_ns: float) -> np.ndarray:
    t_rel = np.arange(num_bins) * bin_size
    tau = (pulse_len_ns * 0.5) / 3.5
    return (t_rel / tau) ** 2 * np.exp(-t_rel / tau)


def waveform_metrics(t: np.ndarray, y: np.ndarray) -> Dict[str, float]:
    peak = y.max()
    area = np.trapezoid(y, t)
    t_peak = t[y.argmax()]
    half = peak * 0.5
    above = y >= half
    if above.any():
        idx = np.where(above)[0]
        fwhm = t[idx[-1]] - t[idx[0]]
    else:
        fwhm = float("nan")
    eff_width = area / peak if peak > 0 else float("nan")
    y_sum = y.sum()
    if y_sum > 0:
        y_norm = y / y_sum
        mean_t = np.sum(t * y_norm)
        var_t = np.sum((t - mean_t) ** 2 * y_norm)
        std_t = float(np.sqrt(var_t))
    else:
        std_t = float("nan")
    return {
        "peak": peak,
        "area": area,
        "fwhm": fwhm,
        "eff_width": eff_width,
        "std": std_t,
        "t_peak": t_peak,
    }


def parse_row(line: str) -> np.ndarray:
    return np.array([float(x) for x in line.split()])


PULSE_LEN_NS = 4.0


def process_dataset(
    name: str,
    cart_path: Path,
    polar_path: Path,
    out_dir: Path,
    pulse_len_ns: float = PULSE_LEN_NS,
) -> None:
    print(f"\n=== {name} ===")
    csv_rows: List[str] = [
        "row,sampling,fwhm_emit,fwhm_recv,eff_emit,eff_recv,std_emit,std_recv,peak_emit,peak_recv,peak_shift,min_time,max_time"
    ]
    best_idx = None
    best_fwhm = -1.0
    best_cart = None
    best_polar = None
    # For mean resampling
    target_n = 200
    t_c_rel_list, t_p_rel_list = [], []
    rec_c_list, rec_p_list = [], []
    emit_c_list, emit_p_list = [], []

    # Load all rows and map by fullwaveIndex to match pulses
    cart_lines = cart_path.read_text().splitlines()
    polar_lines = polar_path.read_text().splitlines()
    cart_map = {float(line.split()[0]): line for line in cart_lines}
    polar_map = {float(line.split()[0]): line for line in polar_lines}
    common_ids = sorted(set(cart_map.keys()) & set(polar_map.keys()))

    if not common_ids:
        print("No matching fullwaveIndex between Cartesian and Polar for", name)
        return

    for idx, fid in enumerate(common_ids):
        rc = parse_row(cart_map[fid])
        rp = parse_row(polar_map[fid])
        rec_c = rc[10:]
        rec_p = rp[10:]
        t_c = np.linspace(rc[7], rc[8], rec_c.size)
        t_p = np.linspace(rp[7], rp[8], rec_p.size)
        bin_c = (
            (rc[8] - rc[7]) / (rec_c.size - 1)
            if rec_c.size > 1
            else pulse_len_ns / 10.0
        )
        bin_p = (
            (rp[8] - rp[7]) / (rec_p.size - 1)
            if rec_p.size > 1
            else pulse_len_ns / 10.0
        )
        emit_c = emitted_wave(rec_c.size, bin_c, pulse_len_ns)
        emit_p = emitted_wave(rec_p.size, bin_p, pulse_len_ns)
        emit_c *= rec_c.max() / emit_c.max()
        emit_p *= rec_p.max() / emit_p.max()

        m_ec = waveform_metrics(t_c, emit_c)
        m_rc = waveform_metrics(t_c, rec_c)
        m_ep = waveform_metrics(t_p, emit_p)
        m_rp = waveform_metrics(t_p, rec_p)

        peak_shift_c = m_rc["t_peak"] - m_ec["t_peak"]
        peak_shift_p = m_rp["t_peak"] - m_ep["t_peak"]

        csv_rows.append(
            f"{int(fid)},cartesian,{m_ec['fwhm']:.6f},{m_rc['fwhm']:.6f},{m_ec['eff_width']:.6f},{m_rc['eff_width']:.6f},"
            f"{m_ec['std']:.6f},{m_rc['std']:.6f},{m_ec['t_peak']:.6f},{m_rc['t_peak']:.6f},{peak_shift_c:.6f},{rc[7]:.6f},{rc[8]:.6f}"
        )
        csv_rows.append(
            f"{int(fid)},polar,{m_ep['fwhm']:.6f},{m_rp['fwhm']:.6f},{m_ep['eff_width']:.6f},{m_rp['eff_width']:.6f},"
            f"{m_ep['std']:.6f},{m_rp['std']:.6f},{m_ep['t_peak']:.6f},{m_rp['t_peak']:.6f},{peak_shift_p:.6f},{rp[7]:.6f},{rp[8]:.6f}"
        )

        max_fwhm_pair = max(m_rc["fwhm"], m_rp["fwhm"])
        if max_fwhm_pair > best_fwhm:
            best_fwhm = max_fwhm_pair
            best_idx = fid
            best_cart = rc
            best_polar = rp

        # collect for mean
        t_c_rel = t_c - t_c.min()
        t_p_rel = t_p - t_p.min()
        t_c_rel_list.append(t_c_rel)
        t_p_rel_list.append(t_p_rel)
        rec_c_list.append(rec_c)
        rec_p_list.append(rec_p)
        emit_c_list.append(emit_c)
        emit_p_list.append(emit_p)

    csv_path = cart_path.parent.parent / f"{name.lower().replace(' ', '_')}_metrics.csv"
    csv_path.write_text("\n".join(csv_rows))
    print(f"Wrote metrics CSV: {csv_path} (rows: {len(csv_rows)-1})")
    print(f"Best matched pulse by received FWHM: {best_idx} (FWHM {best_fwhm:.3f} ns)")

    if best_cart is None or best_polar is None:
        print("No best pulse found; skipping plot for", name)
        return

    rc = best_cart
    rp = best_polar
    rec_c = rc[10:]
    rec_p = rp[10:]
    t_c = np.linspace(rc[7], rc[8], rec_c.size)
    t_p = np.linspace(rp[7], rp[8], rec_p.size)
    bin_c = (
        (rc[8] - rc[7]) / (rec_c.size - 1) if rec_c.size > 1 else pulse_len_ns / 10.0
    )
    bin_p = (
        (rp[8] - rp[7]) / (rec_p.size - 1) if rec_p.size > 1 else pulse_len_ns / 10.0
    )
    emit_c = emitted_wave(rec_c.size, bin_c, pulse_len_ns)
    emit_p = emitted_wave(rec_p.size, bin_p, pulse_len_ns)
    emit_c *= rec_c.max() / emit_c.max()
    emit_p *= rec_p.max() / emit_p.max()

    m_ec = waveform_metrics(t_c, emit_c)
    m_rc = waveform_metrics(t_c, rec_c)
    m_ep = waveform_metrics(t_p, emit_p)
    m_rp = waveform_metrics(t_p, rec_p)

    def fmt(label: str, m) -> str:
        return f"{label} (FWHM {m['fwhm']:.2f} ns, eff {m['eff_width']:.2f} ns)"

    t_c_rel = t_c - t_c.min()
    t_p_rel = t_p - t_p.min()

    plt.figure(figsize=(9, 5))
    # Draw Cartesian first, then Polar to ensure both are visible
    plt.plot(
        t_c_rel,
        emit_c,
        label="Emitted waveform (Cartesian)",
        lw=2.2,
        color="C0",
        zorder=2,
    )
    plt.plot(
        t_c_rel,
        rec_c,
        label=fmt("Received waveform (Cartesian)", m_rc),
        alpha=0.9,
        lw=2,
        color="C1",
        zorder=2,
    )
    plt.plot(
        t_p_rel,
        emit_p,
        label="Emitted waveform (Polar)",
        lw=2.2,
        linestyle="--",
        color="C2",
        zorder=3,
    )
    plt.plot(
        t_p_rel,
        rec_p,
        label=fmt("Received waveform (Polar)", m_rp),
        alpha=0.95,
        lw=2,
        linestyle="--",
        color="C3",
        zorder=3,
    )
    plt.xlabel("Time [ns] (relative)")
    plt.ylabel("Energy / Signal")
    plt.title(f"{name} | Cartesian vs Polar")
    plt.legend()
    plt.grid(alpha=0.15)
    plt.tight_layout()
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{name.lower().replace(' ', '_')}_cartesian_vs_polar.png"
    plt.savefig(out_path, dpi=150)
    print(f"Saved plot: {out_path}")

    # Mean waveforms (resampled to common grids)
    dur_c_mean = float(np.mean([t[-1] for t in t_c_rel_list]))
    dur_p_mean = float(np.mean([t[-1] for t in t_p_rel_list]))
    common_t_c = np.linspace(0, dur_c_mean, target_n)
    common_t_p = np.linspace(0, dur_p_mean, target_n)

    mean_emit_c = np.mean(
        [
            np.interp(common_t_c, t_c_rel_list[i], emit_c_list[i], left=0.0, right=0.0)
            for i in range(len(t_c_rel_list))
        ],
        axis=0,
    )
    mean_rec_c = np.mean(
        [
            np.interp(common_t_c, t_c_rel_list[i], rec_c_list[i], left=0.0, right=0.0)
            for i in range(len(t_c_rel_list))
        ],
        axis=0,
    )
    mean_emit_p = np.mean(
        [
            np.interp(common_t_p, t_p_rel_list[i], emit_p_list[i], left=0.0, right=0.0)
            for i in range(len(t_p_rel_list))
        ],
        axis=0,
    )
    mean_rec_p = np.mean(
        [
            np.interp(common_t_p, t_p_rel_list[i], rec_p_list[i], left=0.0, right=0.0)
            for i in range(len(t_p_rel_list))
        ],
        axis=0,
    )

    m_ec_mean = waveform_metrics(common_t_c, mean_emit_c)
    m_rc_mean = waveform_metrics(common_t_c, mean_rec_c)
    m_ep_mean = waveform_metrics(common_t_p, mean_emit_p)
    m_rp_mean = waveform_metrics(common_t_p, mean_rec_p)

    plt.figure(figsize=(9, 5))
    plt.plot(
        common_t_c,
        mean_emit_c,
        label="Mean emitted (Cartesian)",
        lw=2.2,
        color="C0",
        zorder=2,
    )
    plt.plot(
        common_t_c,
        mean_rec_c,
        label=fmt("Mean received (Cartesian)", m_rc_mean),
        alpha=0.9,
        lw=2,
        color="C1",
        zorder=2,
    )
    plt.plot(
        common_t_p,
        mean_emit_p,
        label="Mean emitted (Polar)",
        lw=2.2,
        linestyle="--",
        color="C2",
        zorder=3,
    )
    plt.plot(
        common_t_p,
        mean_rec_p,
        label=fmt("Mean received (Polar)", m_rp_mean),
        alpha=0.95,
        lw=2,
        linestyle="--",
        color="C3",
        zorder=3,
    )
    plt.xlabel("Time [ns] (relative)")
    plt.ylabel("Energy / Signal")
    plt.title(f"{name} | Mean waveforms (Cartesian vs Polar)")
    plt.legend()
    plt.grid(alpha=0.15)
    plt.tight_layout()
    out_mean = out_dir / f"{name.lower().replace(' ', '_')}_mean_cartesian_vs_polar.png"
    plt.savefig(out_mean, dpi=150)
    print(f"Saved mean plot: {out_mean}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare emitted/received waveforms across datasets."
    )
    parser.add_argument(
        "--base-dir",
        type=Path,
        default=Path("jan-meeting/point_clouds-const_energy"),
        help="Base directory containing Cartesian/ and Polar/ subfolders.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("jan-meeting/temporal_broadening"),
        help="Directory to write plots.",
    )
    parser.add_argument(
        "--cart-subdir",
        type=str,
        default="Cartesian",
        help="Subdir name for Cartesian sampling.",
    )
    parser.add_argument(
        "--polar-subdir",
        type=str,
        default="Polar",
        help="Subdir name for Polar sampling.",
    )
    args = parser.parse_args()

    base = args.base_dir
    datasets = [
        (
            "Plane",
            base / args.cart_subdir / "plane/leg000_fullwave.txt",
            base / args.polar_subdir / "plane/leg000_fullwave.txt",
        ),
        (
            "Plane Tilt",
            base / args.cart_subdir / "plane_tilt/leg000_fullwave.txt",
            base / args.polar_subdir / "plane_tilt/leg000_fullwave.txt",
        ),
        (
            "Step",
            base / args.cart_subdir / "step/leg000_fullwave.txt",
            base / args.polar_subdir / "step/leg000_fullwave.txt",
        ),
        (
            "Cube",
            base / args.cart_subdir / "cube/leg000_fullwave.txt",
            base / args.polar_subdir / "cube/leg000_fullwave.txt",
        ),
    ]
    for name, cart, polar in datasets:
        process_dataset(name, cart, polar, args.out_dir)


if __name__ == "__main__":
    main()
