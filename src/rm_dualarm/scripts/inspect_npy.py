#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
inspect_npy.py - 快速查看包含 dict 的 .npy 数据集
"""

import argparse
import numpy as np
import random
import matplotlib.pyplot as plt
import os

from numpy_pickle_compat import install_numpy_core_aliases

install_numpy_core_aliases()

def prepare_plot_array(values):
    values = np.array(values)
    if values.ndim == 1:
        if values.size == 7:
            arr = values[:, None]
        else:
            arr = values[np.newaxis, :]
    elif values.ndim == 2:
        if values.shape[0] == 7:
            arr = values
        elif values.shape[1] == 7:
            arr = values.T
        elif values.size % 7 == 0:
            arr = values.flatten().reshape(7, -1)
        else:
            flat = values.flatten()
            padded = np.zeros(((7 - (flat.size % 7)) % 7) + flat.size, dtype=flat.dtype)
            padded[:flat.size] = flat
            arr = padded.reshape(7, -1)
    else:
        flat = values.flatten()
        padded = np.zeros(((7 - (flat.size % 7)) % 7) + flat.size, dtype=flat.dtype)
        padded[:flat.size] = flat
        arr = padded.reshape(7, -1)
    return arr


def plot_trajectory(values, title, ylabel, save_prefix=None, extra_avg=False):
    arr = prepare_plot_array(values)
    extra_avg_line = None
    extra_grad_line = None
    if extra_avg:
        avg_line = np.mean(arr, axis=0)
        extra_avg_line = avg_line
        extra_grad_line = np.gradient(avg_line)

    num_steps = arr.shape[1]
    num_subplots = arr.shape[0] + (2 if extra_avg else 0)
    fig, axes = plt.subplots(num_subplots, 1, figsize=(10, 2 * num_subplots), sharex=True)
    if num_subplots == 1:
        axes = [axes]
    fig.suptitle(title)

    for dim in range(arr.shape[0]):
        axes[dim].plot(range(num_steps), arr[dim], marker='o', markersize=3)
        axes[dim].set_ylabel(f"dim {dim}")
        axes[dim].grid(True)

    if extra_avg:
        avg_ax = axes[arr.shape[0]]
        avg_ax.plot(range(num_steps), extra_avg_line, marker='o', markersize=3)
        avg_ax.set_ylabel('avg')
        avg_ax.grid(True)

        grad_ax = axes[arr.shape[0] + 1]
        grad_ax.plot(range(num_steps), extra_grad_line, marker='o', markersize=3)
        grad_ax.set_ylabel('avg grad')
        grad_ax.grid(True)

    axes[-1].set_xlabel('timestep')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if save_prefix:
        plt.savefig(f"{save_prefix}_{ylabel}.png")
    plt.show()


def plot_sample_trajectory(sample, plot_action=True, plot_logprobs=True, save_prefix=None):
    if plot_action and 'action' in sample:
        print("�� Plotting action trajectory for sample...")
        plot_trajectory(sample['action'], "Action vs Timestep", "action", save_prefix)
    elif plot_action:
        print("⚠️ No 'action' key found in sample; skipping action plot.")

    if plot_logprobs and 'action_logprobs' in sample:
        print("�� Plotting action logprobs trajectory for sample...")
        plot_trajectory(sample['action_logprobs'], "Action LogProbs vs Timestep", "action_logprobs", save_prefix, extra_avg=True)
    elif plot_logprobs:
        print("⚠️ No 'action_logprobs' key found in sample; skipping logprobs plot.")


def extract_entire_trajectory(data, key):
    if len(data) == 0:
        return None
    values = [np.array(sample.get(key, [])) for sample in data if key in sample]
    if not values:
        return None

    first = values[0]
    if first.ndim == 1 and first.size == 7:
        stacked = np.stack(values, axis=0)
        return stacked

    if first.ndim == 2 and first.shape[1] == 7:
        concatenated = np.concatenate([np.array(v) for v in values], axis=0)
        return concatenated

    if first.ndim == 2 and first.shape[0] == 7:
        concatenated = np.concatenate([np.array(v) for v in values], axis=1)
        return concatenated

    flat = np.concatenate([v.flatten() for v in values], axis=0)
    if flat.size == 0:
        return None
    if flat.size % 7 == 0:
        return flat.reshape(-1, 7)
    return flat


def plot_entire_trajectory(data, plot_action=True, plot_logprobs=True, save_prefix=None):
    if plot_action:
        action_values = extract_entire_trajectory(data, 'action')
        if action_values is not None and action_values.size > 0:
            print(f"�� Plotting entire action trajectory to {save_prefix}_action.png...")
            plot_trajectory(action_values, "Entire Action Trajectory", "action", save_prefix)
        else:
            print("⚠️ No valid 'action' values found for entire trajectory.")

    if plot_logprobs:
        logprob_values = extract_entire_trajectory(data, 'action_logprobs')
        if logprob_values is not None and logprob_values.size > 0:
            print("�� Plotting entire action_logprobs trajectory...")
            plot_trajectory(logprob_values, "Entire Action LogProbs Trajectory", "action_logprobs", save_prefix, extra_avg=True)
        else:
            print("⚠️ No valid 'action_logprobs' values found for entire trajectory.")


def inspect_npy(npy_path, show_samples=3, plot_sample=None, plot_trajectory=False, plot_action=True, plot_logprobs=True, save_prefix=None):
    # 加载数据
    print(f"�� Loading {npy_path} ...")
    data = np.load(npy_path, allow_pickle=True)
    print(f"✅ Loaded {len(data)} samples")

    # 打印结构信息
    print("\n=== Basic Info ===")
    print("Type:", type(data))
    if len(data) > 0:
        first = data[0]
        print("Sample type:", type(first))
        print("Keys:", list(first.keys()))
        print("Key types:")
        for key, value in first.items():
            try:
                arr = np.array(value)
                print(f"  {key}: {type(value)} shape={arr.shape} dtype={arr.dtype}")
            except Exception:
                print(f"  {key}: {type(value)}")

        # 打印 action 的形状
        try:
            print("Action shape:", np.array(first['action']).shape)
        except Exception:
            print("Action shape: <unknown>")

    # 随机展示若干条样本
    if show_samples > 0:
        print(f"\n=== Showing {show_samples} random samples ===")
        indices = random.sample(range(len(data)), min(show_samples, len(data)))
        for idx in indices:
            sample = data[idx]
            print(f"\n--- Sample {idx} ---")
            print("Prompt:", sample.get('prompt', '<no prompt>'))
            act = np.array(sample.get('action', []))
            print("Action shape:", act.shape)
            print("Action (first few values):", act.flatten()[:10])

            img = sample.get('image', None)
            if img is not None:
                img = np.array(img)
                plt.imshow(img)
                plt.title(sample.get('prompt', f"sample {idx}"))
                plt.axis('off')
                plt.show()

    if plot_trajectory:
        print(f"\n=== Plotting entire trajectory from {npy_path} ===")
        plot_entire_trajectory(data, plot_action=plot_action, plot_logprobs=plot_logprobs, save_prefix=save_prefix)
    elif plot_sample is not None:
        if plot_sample < 0 or plot_sample >= len(data):
            print(f"⚠️ plot_sample index {plot_sample} out of range [0, {len(data) - 1}]")
        else:
            sample = data[plot_sample]
            print(f"\n=== Plotting sample {plot_sample} ===")
            plot_sample_trajectory(sample, plot_action=plot_action, plot_logprobs=plot_logprobs, save_prefix=save_prefix)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Inspect a .npy dataset with dict samples.")
    parser.add_argument("npy_path", type=str, help="Path to the .npy file")
    parser.add_argument("--show", type=int, default=3, help="Number of random samples to display")
    parser.add_argument("--plot-sample", type=int, default=None, help="Sample index to plot trajectory for")
    parser.add_argument("--plot-trajectory", action='store_true', help="Plot the entire trajectory across all samples")
    parser.add_argument("--no-action-plot", action='store_true', help="Disable action trajectory plotting")
    parser.add_argument("--no-logprobs-plot", action='store_true', help="Disable action logprobs plotting")
    parser.add_argument("--save-prefix", type=str, default=None, help="Prefix for saving plot images")
    parser.add_argument("--plot-path", type=str, default="visualize", help="Path to save plot images")
    args = parser.parse_args()
    
    os.makedirs(args.plot_path, exist_ok=True)
    if args.save_prefix is None:
        pref = os.path.join(args.plot_path, "traj" if args.plot_trajectory else "sample")
    else:
        pref = os.path.join(args.plot_path, args.save_prefix)

    inspect_npy(
        args.npy_path,
        args.show,
        plot_sample=args.plot_sample,
        plot_trajectory=args.plot_trajectory,
        plot_action=not args.no_action_plot,
        plot_logprobs=not args.no_logprobs_plot,
        save_prefix=pref,
    )
