# RM75 RLDS 数据处理说明

RM75 工作区里有两套运行环境：

- 机械臂、ROS bag、`npy_convert`、`inspect_npy`：使用 `rm75` conda 环境。
- TFDS/RLDS builder、dataset transform：使用 `rlds_env` conda 环境。

## 1. rosbag 转 episode.npy

`rlds/npy_convert.sh` 顶部已经写好默认参数，直接运行即可：

```bash
conda activate rm75
source install/setup.bash
bash rlds/npy_convert.sh
```

默认输入为 `bags/single`，默认输出为：

```text
rlds/rm75_single_manip/data/train/<task_slug>/episode_<id>/episode.npy
```

常用参数可以直接改 `rlds/npy_convert.sh` 顶部变量：

```bash
BAG_PATH="bags/single"
OUTPUT_DIR="rlds/rm75_single_manip/data/train"
TASK_DESCRIPTION="teleop task"
CONTROL_HZ="20"
WRITE_VIDEO="1"
APPLY_WHITE_BALANCE="1"
```

也可以用环境变量临时覆盖：

```bash
TASK_DESCRIPTION="put_the_eggplant_into_the_plate" CONTROL_HZ=5 bash rlds/npy_convert.sh
```

如果需要完全手动指定，也可以把参数传给脚本，脚本会原样透传给
`rlds/utils/npy_convert.py`：

```bash
bash rlds/npy_convert.sh bags/single \
  --output rlds/rm75_single_manip/data/train \
  --task-description "put_the_eggplant_into_the_plate" \
  --control-hz 5 \
  --video \
  --apply-white-balance
```

## 2. 检查 episode.npy

`rlds/inspect_npy.sh` 顶部包含默认检查参数：

```bash
DATA_ROOT="rlds/rm75_single_manip/data/train"
EPISODE_PATH=""
SHOW_SAMPLES="3"
PLOT_PATH="visualize"
PLOT_TRAJECTORY="1"
```

```bash
conda activate rm75
bash rlds/inspect_npy.sh
```

无参数时，脚本会自动检查 `rlds/rm75_single_manip/data/train` 下找到的第一个
`episode.npy`。也可以显式指定文件：

```bash
bash rlds/inspect_npy.sh \
  rlds/rm75_single_manip/data/train/put_the_eggplant_into_the_plate/episode_0/episode.npy \
  --show 5 \
  --plot-trajectory \
  --plot-path visualize
```

## 3. 构建 TFDS RLDS 数据集

`rlds/tfds_build.sh` 顶部包含默认 build 参数：

```bash
DATASET_DIR="rm75_single_manip"
TFDS_DATA_DIR="datasets"
OVERWRITE="1"
TRY_DOWNLOAD_GCS="0"
MAX_EXAMPLES_PER_SPLIT=""
TRAIN_GLOB=""
VAL_GLOB=""
```

```bash
conda activate rlds_env
bash rlds/tfds_build.sh
```

默认执行 `tfds build --overwrite`。builder 默认读取：

脚本默认会传入 `--download_config '{"try_download_gcs": false}'`，并设置
`NO_GCE_CHECK=true` 和 `TF_CPP_MIN_LOG_LEVEL=3`，避免 TFDS/TensorFlow 在内网环境
访问 Google Cloud、探测 GCE metadata，或输出无关初始化日志。如果确实要允许从
TFDS GCS 复用已发布数据，可设置：

```bash
TRY_DOWNLOAD_GCS=1 bash rlds/tfds_build.sh
```

```text
rlds/rm75_single_manip/data/train/**/episode.npy
rlds/../convert_result/**/episode.npy
rlds/rm75_single_manip/data/val/**/episode.npy
```

如果数据不在默认位置，可以用 glob 环境变量覆盖：

```bash
TRAIN_GLOB="/data/rm75/train/**/episode.npy" \
VAL_GLOB="/data/rm75/val/**/episode.npy" \
bash rlds/tfds_build.sh
```

## 4. 测试 dataset transform

```bash
conda activate rlds_env
export PYTHONPATH="$PWD/rlds:${PYTHONPATH:-}"
python3 rlds/rlds_dataset_builder/test_dataset_transform.py rm75_single_manip \
  --transform-module rm75_single_manip.rm75_single_manip_dataset_transform
```

`rm75_single_manip_dataset_transform.py` 输出训练目标格式：

- `observation.image`: `128x128x3` RGB `uint8`
- `action`: 8 维 `[dx, dy, dz, droll, dpitch, dyaw, gripper, terminate]`
- RLDS step fields: `discount`、`reward`、`is_first`、`is_last`、`is_terminal`
- language fields: `language_instruction`、`language_embedding`
