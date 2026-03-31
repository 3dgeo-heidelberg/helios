#!/usr/bin/env bash

# This script executes the benchmark comparison workflow for two different branches using the google benchmark tools.
# For more information see README.md

is_sourced() {
	[[ "${BASH_SOURCE[0]}" != "$0" ]]
}

die() {
	printf 'Error: %s\n' "$*" >&2
	if is_sourced; then
		return 1
	fi
	exit 1
}

require_cmd() {
	command -v "$1" >/dev/null 2>&1 || die "Required command not found in PATH: $1"
}

run_comparison() {
	(
		set -euo pipefail

		prog_name="${BASH_SOURCE[0]##*/}"

		require_cmd python3
		require_cmd git
		require_cmd cmake
		require_cmd make
		require_cmd nproc

		sanitize_branch_for_filename() {
			local branch_name="$1"
			branch_name="${branch_name//\//_}"
			branch_name="${branch_name//[[:space:]]/_}"
			printf '%s' "$branch_name"
		}

		require_clean_tracked_tree() {
			# Avoid failing on untracked benchmark output JSON files.
			git diff --quiet --ignore-submodules -- || die "Working tree has unstaged changes. Commit/stash them before running: git stash -u"
			git diff --cached --quiet --ignore-submodules -- || die "Working tree has staged changes. Commit/stash them before running: git stash -u"
		}

		# parse the command line arguments
		jobs="$(nproc)"
		repetitions=30
		positional=()

		while [[ $# -gt 0 ]]; do
			case "$1" in
				-j) jobs="$2"; shift 2 ;;
				-j*) jobs="${1#-j}"; shift ;;   # supports -j8
				-r) repetitions="$2"; shift 2 ;;
				-r*) repetitions="${1#-r}"; shift ;;   # supports -r30
				-h|--help) echo "Usage: $prog_name [-j jobs] [-r repetitions] <branch_a> <branch_b>" >&2; exit 0 ;;
				--) shift; positional+=("$@"); break ;;
				-*) echo "Unknown option: $1" >&2; exit 2 ;;
				*) positional+=("$1"); shift ;;
			esac
		done

		set -- "${positional[@]}"
		[[ $# -eq 2 ]] || { echo "Usage: $prog_name [-j jobs] [-r repetitions] <branch_a> <branch_b>" >&2; exit 2; }

		if [[ ! "$repetitions" =~ ^[0-9]+$ ]] || [[ "$repetitions" -lt 1 ]]; then
			die "Invalid -r/--benchmark_repetitions value: '$repetitions' (expected integer >= 1)"
		fi

		branch_a="$1"
		branch_b="$2"
		branch_a_safe="$(sanitize_branch_for_filename "$branch_a")"
		branch_b_safe="$(sanitize_branch_for_filename "$branch_b")"

		echo "Comparing benchmarks for branches: $branch_a and $branch_b with $jobs parallel jobs and $repetitions repetitions"

		# find the benchmark directory where this script is located
		script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
		# find the helios root directory
		repo_root="$(cd -- "$script_dir/.." && pwd)"

		# go to root directory
		cd "$repo_root" || die "Failed to cd into repo root directory: $repo_root"

		require_clean_tracked_tree

		# checkout the two branches and build the benchmarks for each branch
		for branch in "$branch_a" "$branch_b"; do
			require_clean_tracked_tree
			echo "Checking out branch: $branch"
			git checkout "$branch" || die "Failed to checkout branch: $branch"

			branch_safe="$(sanitize_branch_for_filename "$branch")"
			build_dir="$repo_root/build_bench_${branch_safe}"
			echo "Using build directory: $build_dir"

			mkdir -p "$build_dir" || die "Failed to create build directory: $build_dir"
			cd "$build_dir" || die "Failed to cd into build directory: $build_dir"
			# run cmake and build the benchmarks (incremental; directory is preserved across runs)
			cmake -DBUILD_BENCHMARKS=ON -DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer ..
			make -j"$jobs"

			cd .. || die "Failed to cd back to repo root directory"

			# loop over all benchmark executables and run them, saving the results in the compare directory with the branch name in the filename
			for bench_exe in "$build_dir"/benchmarks/*_b; do
				[[ -x "$bench_exe" ]] || continue
				bench_name="$(basename "$bench_exe")"
				echo "Running benchmark: $bench_name for branch: $branch"
				"$bench_exe" --benchmark_repetitions="$repetitions" --benchmark_out_format=json --benchmark_out="benchmarks/compare/${bench_name}_${branch_safe}.json"
			done
		done

		# run the comparison script for every benchmark
		cd benchmarks/compare || die "Failed to cd into compare directory"
		build_dir_a="$repo_root/build_bench_${branch_a_safe}"
		for bench_exe in "$build_dir_a"/benchmarks/*_b; do
			[[ -x "$bench_exe" ]] || continue
			bench_name="$(basename "$bench_exe")"
			json_a="${bench_name}_${branch_a_safe}.json"
			json_b="${bench_name}_${branch_b_safe}.json"
			if [[ ! -f "$json_a" ]]; then
				echo "Skipping comparison for $bench_name: missing $json_a" >&2
				continue
			fi
			if [[ ! -f "$json_b" ]]; then
				echo "Skipping comparison for $bench_name: missing $json_b" >&2
				continue
			fi

			python3 compare.py -d "comparison_output_${bench_name}.json" benchmarks "$json_a" "$json_b"
		done

		echo "Benchmark comparison workflow complete. Results saved in compare directory with filenames: comparison_output_<benchmark_name>.json"
	)
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
	run_comparison "$@"
fi
