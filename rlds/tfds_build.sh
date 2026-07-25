#!/usr/bin/env bash
set -euo pipefail

# Parameters for one-click TFDS build. Edit these defaults in this file, or
# override them with environment variables before running the script.
DATASET_DIR="${DATASET_DIR:-rm75_single_manip}"
TFDS_DATA_DIR="${TFDS_DATA_DIR:-/home/lab-421/workspace/rm_ws/datasets}"
OVERWRITE="${OVERWRITE:-1}"
TRY_DOWNLOAD_GCS="${TRY_DOWNLOAD_GCS:-0}"
MAX_EXAMPLES_PER_SPLIT="${MAX_EXAMPLES_PER_SPLIT:-}"
MANUAL_DIR="${MANUAL_DIR:-}"
DOWNLOAD_DIR="${DOWNLOAD_DIR:-}"
TRAIN_GLOB="${TRAIN_GLOB:-}"
VAL_GLOB="${VAL_GLOB:-}"

if [[ "${CONDA_DEFAULT_ENV:-}" != "rlds_env" ]]; then
  echo "error: activate the rlds_env conda environment first: conda activate rlds_env" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ -z "${TRAIN_GLOB}" ]]; then
  TRAIN_GLOB="${SCRIPT_DIR}/rm75_single_manip/data/train/**/episode.npy:${WORKSPACE_DIR}/convert_result/**/episode.npy"
fi

if [[ -z "${VAL_GLOB}" ]]; then
  VAL_GLOB="${SCRIPT_DIR}/rm75_single_manip/data/val/**/episode.npy"
fi

export CUDA_VISIBLE_DEVICES=""
export NO_GCE_CHECK=true
export TF_CPP_MIN_LOG_LEVEL="${TF_CPP_MIN_LOG_LEVEL:-3}"
export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}"
export RM75_RLDS_TRAIN_GLOB="${TRAIN_GLOB}"
export RM75_RLDS_VAL_GLOB="${VAL_GLOB}"

cd "${SCRIPT_DIR}/${DATASET_DIR}"

if [[ "$#" -eq 0 ]]; then
  args=()

  if [[ "${OVERWRITE}" == "1" ]]; then
    args+=(--overwrite)
  fi

  if [[ "${TRY_DOWNLOAD_GCS}" == "1" ]]; then
    args+=(--download_config '{"try_download_gcs": true}')
  else
    args+=(--download_config '{"try_download_gcs": false}')
  fi

  if [[ -n "${TFDS_DATA_DIR}" ]]; then
    args+=(--data_dir "${TFDS_DATA_DIR}")
  fi

  if [[ -n "${MAX_EXAMPLES_PER_SPLIT}" ]]; then
    args+=(--max_examples_per_split "${MAX_EXAMPLES_PER_SPLIT}")
  fi

  if [[ -n "${MANUAL_DIR}" ]]; then
    args+=(--manual_dir "${MANUAL_DIR}")
  fi

  if [[ -n "${DOWNLOAD_DIR}" ]]; then
    args+=(--download_dir "${DOWNLOAD_DIR}")
  fi

  set -- "${args[@]}"
fi

exec tfds build "$@"
