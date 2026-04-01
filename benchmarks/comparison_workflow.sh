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
		benchmark_filter=""
		positional=()

		while [[ $# -gt 0 ]]; do
			case "$1" in
				-j)
					[[ $# -ge 2 ]] || die "-j requires an argument"
					jobs="$2"; shift 2
					;;
				-j*) jobs="${1#-j}"; shift ;;   # supports -j8
				-r)
					[[ $# -ge 2 ]] || die "-r requires an argument"
					repetitions="$2"; shift 2
					;;
				-r*) repetitions="${1#-r}"; shift ;;   # supports -r30
				-f)
					[[ $# -ge 2 ]] || die "-f requires an argument"
					benchmark_filter="$2"; shift 2
					;;
				-f*) benchmark_filter="${1#-f}"; shift ;;   # supports -fregex
				-h|--help) echo "Usage: $prog_name [-j jobs] [-r repetitions] [-f benchmark_filter] [<branch_a>] <branch_b>" >&2; exit 0 ;;
				--) shift; positional+=("$@"); break ;;
				-*) echo "Unknown option: $1" >&2; exit 2 ;;
				*) positional+=("$1"); shift ;;
			esac
		done

		set -- "${positional[@]}"
		if [[ $# -lt 1 || $# -gt 2 ]]; then
			echo "Usage: $prog_name [-j jobs] [-r repetitions] [-f benchmark_filter] [<branch_a>] <branch_b>" >&2
			exit 2
		fi

		one_branch_mode=0
		branch_a_input=""
		branch_b_input=""
		if [[ $# -eq 1 ]]; then
			one_branch_mode=1
			branch_b_input="$1"
		else
			branch_a_input="$1"
			branch_b_input="$2"
		fi

		if [[ ! "$repetitions" =~ ^[0-9]+$ ]] || [[ "$repetitions" -lt 1 ]]; then
			die "Invalid -r/--benchmark_repetitions value: '$repetitions' (expected integer >= 1)"
		fi

		# find the benchmark directory where this script is located
		script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
		# find the helios root directory
		repo_root="$(cd -- "$script_dir/.." && pwd)"

		# go to root directory
		cd "$repo_root" || die "Failed to cd into repo root directory: $repo_root"

		compare_dir="$repo_root/benchmarks/compare"
		mkdir -p "$compare_dir" || die "Failed to create compare directory: $compare_dir"

		require_clean_tracked_tree

		# clear old benchmark output JSONs before writing new ones
		shopt -s nullglob
		old_json_files=( "$compare_dir"/*.json )
		shopt -u nullglob
		if [[ ${#old_json_files[@]} -gt 0 ]]; then
			echo "Clearing existing benchmark JSON outputs in: $compare_dir"
			rm -f -- "${old_json_files[@]}" || die "Failed to clear existing benchmark JSON outputs in: $compare_dir"
		fi

		if [[ $one_branch_mode -eq 1 ]]; then
			current_branch="$(git -C "$repo_root" symbolic-ref -q --short HEAD || true)"
			if [[ -n "$current_branch" ]]; then
				branch_a="$current_branch"
			else
				branch_a="$(git -C "$repo_root" rev-parse --short HEAD)"
			fi
			branch_b="$branch_b_input"
		else
			branch_a="$branch_a_input"
			branch_b="$branch_b_input"
		fi

		branch_a_safe="$(sanitize_branch_for_filename "$branch_a")"
		branch_b_safe="$(sanitize_branch_for_filename "$branch_b")"

		use_repo_build_dir=0
		repo_build_dir="$repo_root/build"
		if [[ $one_branch_mode -eq 1 ]]; then
			if [[ -d "$repo_build_dir" ]] && [[ -n "$(find "$repo_build_dir" -mindepth 1 -print -quit 2>/dev/null || true)" ]]; then
				use_repo_build_dir=1
			fi
		fi

		if [[ -n "$benchmark_filter" ]]; then
			echo "Comparing benchmarks for branches: $branch_a and $branch_b with $jobs parallel jobs and $repetitions repetitions (filter: $benchmark_filter)"
		else
			echo "Comparing benchmarks for branches: $branch_a and $branch_b with $jobs parallel jobs and $repetitions repetitions"
		fi
		if [[ $use_repo_build_dir -eq 1 ]]; then
			echo "Using existing build directory for current branch '$branch_a': $repo_build_dir (skipping rebuild)"
		fi

		# checkout the two branches and build the benchmarks for each branch
		for branch in "$branch_a" "$branch_b"; do
			require_clean_tracked_tree

			branch_safe="$(sanitize_branch_for_filename "$branch")"
			if [[ $use_repo_build_dir -eq 1 && "$branch" == "$branch_a" ]]; then
				build_dir="$repo_build_dir"
				echo "Using build directory: $build_dir (skipping rebuild)"
			else
				echo "Checking out branch: $branch"
				git checkout "$branch" || die "Failed to checkout branch: $branch"

				build_dir="$repo_root/build_bench_${branch_safe}"
				echo "Using build directory: $build_dir"

				need_build=1
				if [[ -d "$build_dir" ]] && [[ -n "$(find "$build_dir" -mindepth 1 -print -quit 2>/dev/null || true)" ]]; then
					need_build=0
				fi

				if [[ $need_build -eq 1 ]]; then
					mkdir -p "$build_dir" || die "Failed to create build directory: $build_dir"
					cd "$build_dir" || die "Failed to cd into build directory: $build_dir"
					# run cmake and build the benchmarks
					cmake -DBUILD_BENCHMARKS=ON -DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer ..
					make -j"$jobs"
					cd .. || die "Failed to cd back to repo root directory"
				else
					echo "Reusing existing build directory (skipping rebuild): $build_dir"
				fi
			fi

			# loop over all benchmark executables and run them, saving the results in the compare directory with the branch name in the filename
			for bench_exe in "$build_dir"/benchmarks/*_b; do
				[[ -x "$bench_exe" ]] || continue
				bench_name="$(basename "$bench_exe")"
				out_json="$compare_dir/bench__${bench_name}__branch__${branch_safe}.json"
				echo "Running benchmark: $bench_name for branch: $branch"
				if [[ -n "$benchmark_filter" ]]; then
					"$bench_exe" \
						--benchmark_filter="$benchmark_filter" \
						--benchmark_repetitions="$repetitions" \
						--benchmark_out_format=json \
						--benchmark_out="$out_json"
				else
					"$bench_exe" \
						--benchmark_repetitions="$repetitions" \
						--benchmark_out_format=json \
						--benchmark_out="$out_json"
				fi

				# When the filter matches nothing, some benchmark binaries can leave an empty JSON file behind.
				# Delete it so the comparison step only sees valid benchmark JSONs.
				if [[ -f "$out_json" && ! -s "$out_json" ]]; then
					rm -f -- "$out_json" || die "Failed to remove empty benchmark JSON output: $out_json"
					echo "Skipping $bench_name for branch $branch: filter produced no benchmarks" >&2
				fi
			done
		done

		# run the comparison script for every benchmark
		cd benchmarks/compare || die "Failed to cd into compare directory"
		shopt -s nullglob
		json_a_files=( "bench__"*"__branch__${branch_a_safe}.json" )
		shopt -u nullglob
		if [[ ${#json_a_files[@]} -eq 0 ]]; then
			die "No benchmark output JSON files found for branch '$branch_a' (expected files like: bench__<benchmark>__branch__${branch_a_safe}.json)"
		fi

		for json_a in "${json_a_files[@]}"; do
			if [[ ! -s "$json_a" ]]; then
				echo "Skipping comparison: empty JSON output $json_a" >&2
				continue
			fi

			bench_name="${json_a#bench__}"
			bench_name="${bench_name%__branch__${branch_a_safe}.json}"
			json_b="bench__${bench_name}__branch__${branch_b_safe}.json"
			if [[ ! -f "$json_b" ]]; then
				echo "Skipping comparison for $bench_name: missing $json_b" >&2
				continue
			fi
			if [[ ! -s "$json_b" ]]; then
				echo "Skipping comparison for $bench_name: empty $json_b" >&2
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
