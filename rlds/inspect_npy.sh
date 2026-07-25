#!/usr/bin/env bash
set -euo pipefail

# Parameters for one-click inspection. Set EPISODE_PATH to inspect a specific
# file; otherwise the first train episode under DATA_ROOT is used.
DATA_ROOT="${DATA_ROOT:-rlds/rm75_single_manip/data/train}"
EPISODE_PATH="${EPISODE_PATH:-}"
SHOW_SAMPLES="${SHOW_SAMPLES:-3}"
PLOT_PATH="${PLOT_PATH:-visualize}"
SAVE_PREFIX="${SAVE_PREFIX:-}"
PLOT_TRAJECTORY="${PLOT_TRAJECTORY:-1}"
PLOT_ACTION="${PLOT_ACTION:-1}"
PLOT_LOGPROBS="${PLOT_LOGPROBS:-0}"

if [[ "${CONDA_DEFAULT_ENV:-}" != "rm75" ]]; then
  echo "error: activate the rm75 conda environment first: conda activate rm75" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WORKSPACE_DIR}"

if [[ "$#" -eq 0 ]]; then
  if [[ -z "${EPISODE_PATH}" ]]; then
    EPISODE_PATH="$(find "${DATA_ROOT}" -path '*/episode.npy' -type f 2>/dev/null | sort | head -n 1 || true)"
  fi
  if [[ -z "${EPISODE_PATH}" ]]; then
    echo "error: no episode.npy found under ${DATA_ROOT}" >&2
    exit 1
  fi

  args=("${EPISODE_PATH}" --show "${SHOW_SAMPLES}" --plot-path "${PLOT_PATH}")

  if [[ "${PLOT_TRAJECTORY}" == "1" ]]; then
    args+=(--plot-trajectory)
  fi

  if [[ "${PLOT_ACTION}" != "1" ]]; then
    args+=(--no-action-plot)
  fi

  if [[ "${PLOT_LOGPROBS}" != "1" ]]; then
    args+=(--no-logprobs-plot)
  fi

  if [[ -n "${SAVE_PREFIX}" ]]; then
    args+=(--save-prefix "${SAVE_PREFIX}")
  fi

  set -- "${args[@]}"
fi

exec python3 "${SCRIPT_DIR}/utils/inspect_npy.py" "$@"
