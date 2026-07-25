"""Training transform for the RM75 single-arm manipulation RLDS dataset."""

from __future__ import annotations

from typing import Any, Dict

import numpy as np
from PIL import Image


TARGET_IMAGE_SIZE = (224, 224)
SOURCE_ACTION_DIM = 7
TARGET_ACTION_DIM = 8


def _resize_image(image: Any) -> np.ndarray:
    array = np.asarray(image)
    if array.ndim != 3 or array.shape[-1] != 3:
        raise ValueError(f"expected RGB image with shape HxWx3, got {array.shape}")
    if array.dtype != np.uint8:
        array = np.clip(array, 0, 255).astype(np.uint8)
    resized = Image.fromarray(array).resize(TARGET_IMAGE_SIZE, Image.Resampling.LANCZOS)
    return np.asarray(resized, dtype=np.uint8)


def _terminate_action(step: Dict[str, Any]) -> np.ndarray:
    source_action = np.asarray(step["action"], dtype=np.float32).reshape(-1)
    if source_action.size != SOURCE_ACTION_DIM:
        raise ValueError(f"expected {SOURCE_ACTION_DIM}-D action, got shape {source_action.shape}")
    terminate = np.asarray([1.0 if bool(step["is_last"]) else 0.0], dtype=np.float32)
    action = np.concatenate([source_action, terminate]).astype(np.float32)
    if action.size != TARGET_ACTION_DIM:
        raise ValueError(f"expected {TARGET_ACTION_DIM}-D transformed action, got {action.shape}")
    return action


def transform_step(step: Dict[str, Any]) -> Dict[str, Any]:
    """Maps an RM75 source step to the training target spec.

    Output action layout:
      [dx, dy, dz, droll, dpitch, dyaw, gripper, terminate]
    """
    transformed_step = {
        "observation": {
            "image": _resize_image(step["observation"]["image"]),
        },
        "action": _terminate_action(step),
    }

    for copy_key in (
        "discount",
        "reward",
        "is_first",
        "is_last",
        "is_terminal",
        "language_instruction",
        "language_embedding",
    ):
        transformed_step[copy_key] = step[copy_key]

    return transformed_step
