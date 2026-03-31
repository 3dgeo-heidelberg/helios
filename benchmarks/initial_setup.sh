#!/usr/bin/env bash

# This script provides the initial setup for the benchmark comparison workflow.
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

run_setup() {
	require_cmd wget

	echo "Setting up the benchmark comparison workflow..."

	# find the benchmark directory where this script is located
	script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

	# Work inside the benchmark directory where this script lives
	cd "$script_dir" || die "Failed to cd into script directory: $script_dir"

	# create a directory for the results
	mkdir -p compare/gbench

	# download the script files from the google benchmark tools github
	cd compare || die "Failed to cd into compare directory"

	wget -N https://raw.githubusercontent.com/google/benchmark/refs/heads/main/tools/compare.py
	wget -N https://raw.githubusercontent.com/google/benchmark/refs/heads/main/tools/requirements.txt

	cd gbench || die "Failed to cd into gbench directory"

	wget -N https://raw.githubusercontent.com/google/benchmark/refs/heads/main/tools/gbench/report.py
	wget -N https://raw.githubusercontent.com/google/benchmark/refs/heads/main/tools/gbench/util.py

	cd .. || die "Failed to cd back to compare directory"

	echo "Benchmark comparison workflow setup complete."
	echo "To use the workflow, activate your conda environment and run comparison_workflow.sh"
}

if is_sourced; then
	# Avoid changing the caller's current directory when sourced.
	( run_setup )
else
	set -euo pipefail
	run_setup
fi
