#!/usr/bin/env python3
"""
Sweep beamSampleQuality (N) and compare mean waveform energy for Cartesian vs Polar.

What it does:
 1) For each scene and each N in the sweep, copy the Cartesian and Polar survey XMLs,
    set FWFSettings@beamSampleQuality=N, and run helios++.
 2) Read leg000_fullwave.txt, compute mean waveform energy (mean over all samples,
    across all pulses) for each sampling.
 3) Save CSV of results and a ratio plot (Polar/Cartesian) vs N per scene.

You must provide the scene XML pairs (cart:polar) and a helios++ binary path.

Example:
  python scripts/beam_quality_sweep.py \
    --helios-bin ./build/helios++ \
    --out-dir jan-meeting/temporal_broadening/beam_quality_sweep \
    --scenes \
      plane:test_scenarios/cartesian_plane.xml:test_scenarios/polar_plane.xml \
      plane_tilt:test_scenarios/cartesian_plane_tilt.xml:test_scenarios/polar_plane_tilt.xml \
      step:test_scenarios/cartesian_step.xml:test_scenarios/polar_step.xml \
      cube:test_scenarios/cartesian_cube.xml:test_scenarios/polar_cube.xml
"""
import argparse
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np


def parse_scenes(scene_args: List[str]) -> Dict[str, Tuple[Path, Path]]:
    scenes = {}
    for item in scene_args:
        try:
            name, cart, polar = item.split(":")
        except ValueError:
            raise ValueError(
                f"Invalid --scenes entry '{item}'. Use name:cart.xml:polar.xml"
            )
        scenes[name] = (Path(cart), Path(polar))
    return scenes


def patch_beam_sample(src: Path, dst: Path, beam_sample_quality: int) -> None:
    tree = ET.parse(src)
    root = tree.getroot()
    for fwf in root.findall(".//FWFSettings"):
        fwf.set("beamSampleQuality", str(beam_sample_quality))
    dst.parent.mkdir(parents=True, exist_ok=True)
    tree.write(dst, encoding="UTF-8", xml_declaration=True)


def run_helios(
    bin_path: Path, survey_xml: Path, output_dir: Path, assets_dir: Path
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(bin_path),
        str(survey_xml),
        "--writeWaveform",
        "--assets",
        str(assets_dir),
        "--output",
        str(output_dir),
    ]
    subprocess.run(cmd, check=True)


def mean_waveform_energy(fullwave_path: Path) -> float:
    rows = []
    with fullwave_path.open() as f:
        for line in f:
            parts = [float(x) for x in line.split()]
            if len(parts) > 10:
                rows.append(parts[10:])
    if not rows:
        return float("nan")
    # Pad to max length with zeros so we can stack
    max_len = max(len(r) for r in rows)
    arr = np.zeros((len(rows), max_len))
    for i, r in enumerate(rows):
        arr[i, : len(r)] = r
    return float(arr.mean())


def process_scene(
    name: str,
    cart_xml: Path,
    polar_xml: Path,
    beam_samples: List[int],
    helios_bin: Path,
    out_dir: Path,
    assets_dir: Path,
) -> List[Tuple[int, float, float, float]]:
    results = []
    tmpdir = Path(tempfile.mkdtemp(prefix=f"beamq_{name}_"))
    try:
        for n in beam_samples:
            cart_mod = tmpdir / f"{name}_cart_{n}.xml"
            polar_mod = tmpdir / f"{name}_polar_{n}.xml"
            patch_beam_sample(cart_xml, cart_mod, n)
            patch_beam_sample(polar_xml, polar_mod, n)

            cart_out = out_dir / f"{name}/cartesian/N{n}"
            polar_out = out_dir / f"{name}/polar/N{n}"
            run_helios(helios_bin, cart_mod, cart_out, assets_dir)
            run_helios(helios_bin, polar_mod, polar_out, assets_dir)

            # find leg000_fullwave.txt recursively (helios nests by survey name/date)
            cart_fw = next(cart_out.rglob("leg000_fullwave.txt"))
            polar_fw = next(polar_out.rglob("leg000_fullwave.txt"))
            cart_mean = mean_waveform_energy(cart_fw)
            polar_mean = mean_waveform_energy(polar_fw)
            ratio = polar_mean / cart_mean if cart_mean else float("nan")
            results.append((n, cart_mean, polar_mean, ratio))
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)
    return results


def plot_ratios(
    all_results: Dict[str, List[Tuple[int, float, float, float]]], out_dir: Path
) -> None:
    plt.figure(figsize=(8, 5))
    for scene, rows in all_results.items():
        rows_sorted = sorted(rows, key=lambda r: r[0])
        Ns = [r[0] for r in rows_sorted]
        ratios = [r[3] for r in rows_sorted]
        plt.plot(Ns, ratios, marker="o", label=scene)
    plt.axhline(1.0, color="gray", linestyle="--", linewidth=1)
    plt.xlabel("beamSampleQuality (N)")
    plt.ylabel("Mean energy ratio (Polar / Cartesian)")
    plt.title("Mean waveform energy ratio vs beamSampleQuality")
    plt.legend()
    plt.grid(alpha=0.2)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "mean_energy_ratio.png"
    plt.savefig(out_path, dpi=150)
    print(f"Saved ratio plot: {out_path}")


def write_csv(
    all_results: Dict[str, List[Tuple[int, float, float, float]]], out_dir: Path
) -> None:
    lines = ["scene,N,cartesian_mean,polar_mean,ratio"]
    for scene, rows in all_results.items():
        for n, c, p, r in rows:
            lines.append(f"{scene},{n},{c:.6f},{p:.6f},{r:.6f}")
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "mean_energy_ratio.csv"
    csv_path.write_text("\n".join(lines))
    print(f"Saved CSV: {csv_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Sweep beamSampleQuality and compare mean waveform energies."
    )
    parser.add_argument(
        "--helios-bin", type=Path, required=True, help="Path to helios++ binary."
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        required=True,
        help="Output directory for plots/CSV and helios outputs.",
    )
    parser.add_argument(
        "--scenes",
        nargs="+",
        required=True,
        help="Scene definitions: name:cart.xml:polar.xml (e.g., plane:cart.xml:polar.xml)",
    )
    parser.add_argument(
        "--beam-samples",
        nargs="+",
        type=int,
        default=[5, 10, 20, 30, 40, 50, 100],
        help="Values to set for FWFSettings@beamSampleQuality.",
    )
    parser.add_argument(
        "--assets-dir",
        type=Path,
        default=Path("test_scenarios"),
        help="Assets directory to pass to helios++.",
    )
    args = parser.parse_args()

    scenes = parse_scenes(args.scenes)
    all_results: Dict[str, List[Tuple[int, float, float, float]]] = {}
    for name, (cart, polar) in scenes.items():
        res = process_scene(
            name,
            cart,
            polar,
            args.beam_samples,
            args.helios_bin,
            args.out_dir,
            args.assets_dir,
        )
        all_results[name] = res

    plot_ratios(all_results, args.out_dir)
    write_csv(all_results, args.out_dir)


if __name__ == "__main__":
    main()
