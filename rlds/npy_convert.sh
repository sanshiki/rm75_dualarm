#!/usr/bin/env bash
set -euo pipefail

# Parameters for one-click conversion. Edit these defaults in this file, or
# override them with environment variables before running the script.
BAG_PATH="${BAG_PATH:-bags/single}"
OUTPUT_DIR="${OUTPUT_DIR:-rlds/rm75_single_manip/data/train}"
TASK_DESCRIPTION="${TASK_DESCRIPTION:-teleop task}"
CONTROL_HZ="${CONTROL_HZ:-20}"
VIDEO_FPS="${VIDEO_FPS:-${CONTROL_HZ}}"
WHITE_BALANCE_CONFIG="${WHITE_BALANCE_CONFIG:-src/rm_dualarm/config/white_balance.yaml}"
ACTION_MIN="${ACTION_MIN:--0.2 -0.2 -0.2 -0.5 -0.5 -0.5 0.0}"
ACTION_MAX="${ACTION_MAX:-0.2 0.2 0.2 0.5 0.5 0.5 1.0}"
ACTION_NORMALIZED_RANGE="${ACTION_NORMALIZED_RANGE:--1.0 1.0}"
DEFAULT_GRIPPER="${DEFAULT_GRIPPER:-1.0}"
GRIPPER_EVENT_MAX_DT="${GRIPPER_EVENT_MAX_DT:-1.0}"
SUCCESS="${SUCCESS:-}"
SCORE="${SCORE:-}"
WRITE_VIDEO="${WRITE_VIDEO:-1}"
APPLY_WHITE_BALANCE="${APPLY_WHITE_BALANCE:-1}"
CLIP_NORMALIZED_ACTION="${CLIP_NORMALIZED_ACTION:-1}"

if [[ "${CONDA_DEFAULT_ENV:-}" != "rm75" ]]; then
  echo "error: activate the rm75 conda environment first: conda activate rm75" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WORKSPACE_DIR}"

if [[ "$#" -eq 0 ]]; then
  read -r -a ACTION_MIN_VALUES <<< "${ACTION_MIN}"
  read -r -a ACTION_MAX_VALUES <<< "${ACTION_MAX}"
  read -r -a ACTION_RANGE_VALUES <<< "${ACTION_NORMALIZED_RANGE}"

  args=(
    "${BAG_PATH}"
    --output "${OUTPUT_DIR}"
    --task-description "${TASK_DESCRIPTION}"
    --control-hz "${CONTROL_HZ}"
    --video-fps "${VIDEO_FPS}"
    --white-balance-config "${WHITE_BALANCE_CONFIG}"
    --action-min "${ACTION_MIN_VALUES[@]}"
    --action-max "${ACTION_MAX_VALUES[@]}"
    --action-normalized-range "${ACTION_RANGE_VALUES[@]}"
    --default-gripper "${DEFAULT_GRIPPER}"
    --gripper-event-max-dt "${GRIPPER_EVENT_MAX_DT}"
  )

  if [[ "${WRITE_VIDEO}" == "1" ]]; then
    args+=(--video)
  else
    args+=(--no-video)
  fi

  if [[ "${APPLY_WHITE_BALANCE}" == "1" ]]; then
    args+=(--apply-white-balance)
  else
    args+=(--no-apply-white-balance)
  fi

  if [[ "${CLIP_NORMALIZED_ACTION}" == "1" ]]; then
    args+=(--clip-normalized-action)
  else
    args+=(--no-clip-normalized-action)
  fi

  if [[ -n "${SUCCESS}" ]]; then
    if [[ "${SUCCESS}" == "1" ]]; then
      args+=(--success)
    else
      args+=(--no-success)
    fi
  fi

  if [[ -n "${SCORE}" ]]; then
    args+=(--score "${SCORE}")
  fi

  set -- "${args[@]}"
fi

exec python3 "${SCRIPT_DIR}/utils/npy_convert.py" "$@"
