# Raw Episode 数据格式说明

本文档定义自采集数据在进入 GRAPE/TPO 后处理之前的 raw episode 格式。这个阶段还不做 chosen/rejected 匹配，也不要求成功轨迹和失败轨迹已经成对。每个目录只表示一次完整执行轨迹，后续脚本再根据成功标记、分数、任务和初始状态去构建 preference pair。

适用输入包括 ROS2 rosbag、仿真 rollout、人工遥操作记录或其他采集系统。转换目标是把每条 episode 统一保存成：

```text
<task_slug>/episode_<episode_id>/
  episode.npy
  episode.mp4
  metadata.json
```

## 目录结构

推荐 raw result 根目录：

```text
data/data_collect/<source>/<date>/raw_results/
  <task_slug>/
    episode_0/
      episode.npy
      episode.mp4
      metadata.json
    episode_1/
      episode.npy
      episode.mp4
      metadata.json
    episode_2/
      episode.npy
      episode.mp4
      metadata.json
```

要求：

- `<task_slug>` 是文件系统安全的任务 ID，建议小写并用下划线连接。
- 每个 `episode_x/` 是一条独立轨迹。
- 每个 episode 目录必须包含 `episode.npy` 和 `metadata.json`。
- 推荐同时保存 `episode.mp4`，用于人工检查、VLM 标注和 FrameSkip 可视化。
- 不要在这个阶段命名为 `success_*.npy` 或 `failure_*.npy`，因为 chosen/rejected 匹配应该由后续 prepare/pairing 阶段完成。

如果你想保留原始 rosbag，也可以放在更上层或写进 `metadata.json`，不建议把大体积 `.db3` 混进每个 episode 目录。

## metadata.json

每个 raw episode 目录必须包含 `metadata.json`。它描述任务、执行结果、数据来源、同步策略和控制约定。

推荐 schema：

```json
{
  "task": {
    "raw_name": "put the spoon on the towel",
    "slug": "put_the_spoon_on_the_towel",
    "prompt": "put the spoon on the towel"
  },
  "episode": {
    "id": 0,
    "file": "episode.npy",
    "video": "episode.mp4",
    "success": true,
    "score": 5.0,
    "num_steps": 42,
    "duration_sec": 8.4
  },
  "source": {
    "type": "ros2_rosbag",
    "rosbag": "/abs/path/to/bag_001.db3",
    "camera_topic": "/camera/image_raw",
    "tf_topic": "/tf",
    "target_pose_topic": "/target_pose",
    "traj_cmd_topic": "/traj_cmd",
    "joint_states_topic": "/joint_states",
    "gripper_cmd_topic": "/gripper_cmd",
    "sync_policy": "control_time_nearest_neighbor",
    "control_hz": 5,
    "tf_frames": {
      "base": "base_link",
      "end_effector": "tool0"
    }
  },
  "action": {
    "type": "ee_delta_pose_gripper",
    "position_unit": "meter",
    "rotation_unit": "radian",
    "frame": "base_link",
    "layout": ["dx", "dy", "dz", "droll", "dpitch", "dyaw", "gripper"],
    "gripper_open_value": 1.0,
    "gripper_closed_value": 0.0
  },
  "image": {
    "encoding": "rgb_uint8",
    "shape": [480, 640, 3],
    "camera_view": "primary"
  }
}
```

字段说明：

- `task.raw_name`：采集系统里的原始任务名。
- `task.slug`：目录名使用的任务 ID。
- `task.prompt`：训练和 eval 时喂给 OpenVLA 的语言指令。
- `episode.id`：episode 编号，应和目录名 `episode_<id>` 一致。
- `episode.file`：轨迹文件名，固定推荐为 `episode.npy`。
- `episode.video`：视频文件名，固定推荐为 `episode.mp4`；没有视频时可为 `null`，但不推荐。
- `episode.success`：这条轨迹是否成功。后续 pairing 会用它区分 success/failure 候选。
- `episode.score`：轨迹级分数，越高越好；可以是人工打分、环境 reward、规则 cost 或 success/failure 派生分数。
- `episode.num_steps`：`episode.npy` 中 timestep 数量。
- `episode.duration_sec`：轨迹持续时间。
- `source`：原始数据来源和同步策略，主要用于回溯和排查。
- `action`：7D action 的语义、单位、坐标系和夹爪约定。
- `image`：保存到 `.npy` 里的图像格式说明。

`episode.success` 和 `episode.score` 可以同时存在。pairing 阶段可以先按 `success` 分组，再用 `score` 选择同任务、同初始状态或相近初始状态下的更优/更差轨迹。

## episode.npy

`episode.npy` 表示一条完整轨迹，保存为 NumPy object array。array 的每个元素是一个 timestep dictionary。

最小必需 schema：

```python
{
    "images": np.ndarray,      # uint8 RGB image, shape (H, W, 3)
    "prompt": str,             # task language instruction
    "action": list[float],     # 7D: dx, dy, dz, droll, dpitch, dyaw, gripper
}
```

后续 dataset builder 最核心会读取：

- `step["images"]`
- `step["prompt"]`
- `step["action"]`
- 可选的 `step["action_logprobs"]`

### images

要求：

- 类型：`np.ndarray`
- dtype：`uint8`
- shape：`(H, W, 3)`
- 通道顺序：RGB
- 像素范围：`[0, 255]`

源图像可以保持采集分辨率，例如 `480x640`。不要在 raw episode 阶段强行改成 `224x224`，除非你明确希望永久丢弃原始视觉分辨率。TFDS build 阶段可以再根据训练/eval pipeline 做 resize。

如果图像来自 OpenCV，注意 OpenCV 默认是 BGR，保存进 `episode.npy` 前必须转换成 RGB。

### prompt

要求：

- 类型：Python `str`
- 同一条 episode 内所有 timestep 通常使用同一个 prompt。
- prompt 应尽量和 eval 时传给模型的 instruction 一致。

示例：

```python
"put the spoon on the towel"
```

### action

要求：

- 长度：`7`
- 可转换为 `np.float32`
- 语义：

```text
action[0:3] = 末端位置增量 dx, dy, dz
action[3:6] = 末端旋转增量 droll, dpitch, dyaw
action[6]   = 夹爪命令
```

推荐单位和约定：

- 位置：米，建议在 robot base frame 下表示。
- 旋转：弧度。
- 夹爪：`1.0 = open`，`0.0 = closed`。

对于 ROS2 rosbag 中的 `/target_pose` 和 `/tf`，推荐在每个控制 timestep 上做时间同步，然后用当前末端位姿和目标位姿计算 6D delta：

```text
current_ee_pose_t = /tf 中的当前末端位姿
target_pose_t     = 同一时刻的 /target_pose
delta_position    = target_position_t - current_position_t
delta_rotation    = rotation_delta(target_orientation_t, current_orientation_t)
gripper           = bool gripper command -> 1.0 open / 0.0 closed
```

如果原始命令来自关节空间 `/traj_cmd`，应先用正运动学把目标关节角转换成目标末端位姿，再计算 6D delta。不要直接把关节角 `[-pi, pi]` 保存成这里的 7D action，除非后续同时修改 action tokenizer、训练 transform 和 eval controller。

### 推荐可选 step 字段

这些字段不属于训练最小必需，但强烈建议 raw episode 阶段保留，方便检查转换是否正确：

```python
{
    "timestamp": float,
    "image_timestamp": float,
    "control_timestamp": float,
    "ee_pose": np.ndarray,          # 当前末端位姿，例如 xyz + quaternion
    "target_pose": np.ndarray,      # 用于计算 action 的目标位姿
    "joint_state": np.ndarray,      # 当前关节角 state
    "traj_cmd": np.ndarray,         # 原始关节或控制命令，如果有
    "gripper_state": float,
    "gripper_cmd": float,
    "action_logprobs": np.ndarray,  # shape (7,)，如果有 teacher policy logprob
    "is_keyframe": bool             # 可选人工或预计算 keyframe 标签
}
```

建议至少保留 `timestamp`、`ee_pose`、`target_pose`、`joint_state` 和 `gripper_cmd`。这些字段可以帮助你确认 action delta 的方向、单位、坐标系和夹爪约定是否正确。

## episode.mp4

每条 `episode.npy` 推荐配套一个同目录下的 `episode.mp4`：

```text
episode.npy
episode.mp4
```

要求：

- 视频帧应和 `episode.npy` timestep 一一对应，或者至少足够接近，便于人工检查和 VLM 标注。
- 视频内容应来自和 `step["images"]` 相同的相机视角。
- FPS 应稳定，并尽量匹配采集或控制频率。WidowX/Simpler-style 数据常用 `5 FPS`，Google Robot-style 数据常用 `3 FPS`。
- 视频应直接反映训练使用的视觉输入，不建议使用额外拼接、多视角合成或和训练 image 不一致的渲染视角。

训练 TFDS 主要从 `.npy` 构建；`.mp4` 主要用于人工检查、VLM milestone 标注、FrameSkip 可视化和排查数据问题。缺失视频不一定阻塞最基础的 dataset build，但会影响后续 annotation 和调试。

## 后续 pairing 阶段

raw episode 本身不区分 chosen/rejected 文件名。后续 prepare/pairing 脚本应该读取多个 raw episode，并根据以下信息构建标准 preference pair：

- `task.slug` 或目录 `<task_slug>` 相同。
- `episode.success` 和 `episode.score` 可用于选择更优和更差轨迹。
- 如果能记录初始状态 ID、场景 ID、物体位姿或 reset seed，应在 `metadata.json` 中额外保存，pairing 时优先匹配相同或相近初始状态。
- pairing 输出阶段再生成 `success_*.npy`、`failure_*.npy`、pair-level `metadata.json` 和后续训练需要的 `results/` 结构。

可选在 `metadata.json` 中增加初始状态字段：

```json
{
  "initial_state": {
    "scene_id": "scene_001",
    "reset_seed": 123,
    "object_poses": {
      "spoon": [0.42, -0.18, 0.03, 0.0, 0.0, 0.0, 1.0]
    }
  }
}
```

## 构建前校验清单

每条 raw episode 至少应满足：

- 目录结构为 `<task_slug>/episode_x/episode.npy`。
- 同目录下存在 `metadata.json`。
- `metadata.json` 中的 `episode.file` 指向 `episode.npy`。
- `episode.npy` 可以用 `np.load(path, allow_pickle=True)` 正常读取。
- 每个 timestep 都包含 `images`、`prompt`、`action`。
- `images` 是 RGB `uint8`，shape 为 `(H, W, 3)`。
- `action` 转成 `np.float32` 后 shape 为 `(7,)`。
- `prompt` 非空，并和 `metadata.json.task.prompt` 一致。
- `episode.success` 是明确的 bool，或者有其他可以判断成功/失败的字段。
- `episode.score` 的含义在同一任务内一致，并且越大越好。
- `episode.mp4` 内容和 `episode.npy` 中的图像序列对应。
- 夹爪约定全数据集一致：`1.0 = open`，`0.0 = closed`。

## 常见错误

- 在 raw episode 阶段就写成 `success_*.npy` / `failure_*.npy`，导致后续 pairing 逻辑和数据来源混在一起。
- 保存了 OpenCV BGR 图像，导致颜色通道错位。
- 把关节角直接当成 7D 末端 action 保存。
- 同一个数据集中混用了绝对目标位姿和 delta action。
- prompt 和 eval 时使用的 instruction 不一致。
- 夹爪 open/close 约定在训练和 eval 中相反。
- `episode.mp4` 帧数和 `episode.npy` timestep 严重不一致，导致后续标注难以对齐。
