# CERBERUS数据集运行指南

## 目录结构

轨迹文件将按照以下结构保存：
```
/ws/results/CERBERUS/
├── stereo+imu/
│   ├── seq0/
│   │   └── trajectory.txt
│   ├── seq1/
│   │   └── trajectory.txt
│   └── ...
├── stereo/
│   ├── seq0/
│   │   └── trajectory.txt
│   └── ...
└── mono+imu/
    ├── seq0/
    │   └── trajectory.txt
    └── ...
```

## 运行方式

### 1. 运行单个序列

使用 `run_cerberus.sh` 脚本：

```bash
# 基本用法（使用默认参数，包含RViz可视化）
./run_cerberus.sh

# 指定模式和序列
./run_cerberus.sh stereo+imu seq0

# 指定完整的bag文件路径
./run_cerberus.sh stereo+imu seq0 /datasets/CERBERUS/seq0.bag

# 关闭可视化（适合无头服务器）
./run_cerberus.sh stereo+imu seq0 /datasets/CERBERUS/seq0.bag false
```

### 2. 批量运行所有序列

使用 `run_cerberus_all.sh` 脚本：

```bash
# 运行所有序列（stereo+imu模式，无可视化）
./run_cerberus_all.sh

# 运行所有序列（指定模式）
./run_cerberus_all.sh stereo
./run_cerberus_all.sh mono+imu

# 批量运行时启用可视化（不推荐，会打开多个RViz窗口）
./run_cerberus_all.sh stereo+imu true
```

### 3. 直接使用roslaunch

```bash
# 运行单个序列（包含RViz可视化）
roslaunch ov_msckf cerberus.launch mode:=stereo+imu seq:=seq0 bag:=/datasets/CERBERUS/seq0.bag

# 运行立体+IMU模式
roslaunch ov_msckf cerberus.launch mode:=stereo+imu seq:=seq1

# 运行纯立体模式
roslaunch ov_msckf cerberus.launch mode:=stereo seq:=seq2

# 运行单目+IMU模式
roslaunch ov_msckf cerberus.launch mode:=mono+imu seq:=seq3

# 关闭可视化
roslaunch ov_msckf cerberus.launch mode:=stereo+imu seq:=seq0 doviz:=false

# 使用自定义RViz配置
roslaunch ov_msckf cerberus.launch mode:=stereo+imu seq:=seq0 rviz_config:=/path/to/your/config.rviz
```

## 参数说明

### 模式参数 (mode)
- `stereo+imu`: 立体相机 + IMU（默认）
- `stereo`: 仅立体相机
- `mono+imu`: 单目相机 + IMU

### 序列参数 (seq)
- `seq0`, `seq1`, `seq2`, ..., `seq7`: CERBERUS数据集的8个序列

### 其他参数
- `bag`: rosbag文件路径
- `dosave`: 是否保存轨迹（默认: true）
- `dotime`: 是否记录时间信息（默认: false）
- `doviz`: 是否启动RViz可视化（默认: true）
- `rviz_config`: RViz配置文件路径（默认: display.rviz）

## 输出文件

每个序列运行后会生成：
- `trajectory.txt`: 轨迹文件（时间戳 + 位姿 + 协方差）
- `timing.txt`: 时间统计文件（如果启用）

## 轨迹文件格式

```
# timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33
1.23456 0.123456 0.234567 0.345678 0.0 0.0 0.0 1.0 0.001 0.0 0.0 0.001 0.0 0.001 0.01 0.0 0.0 0.01 0.0 0.01
```

其中：
- `timestamp`: 时间戳（秒）
- `tx ty tz`: 位置 (x, y, z)
- `qx qy qz qw`: 四元数 (x, y, z, w)
- `Pr11 Pr12 Pr13 Pr22 Pr23 Pr33`: 旋转协方差矩阵上三角
- `Pt11 Pt12 Pt13 Pt22 Pt23 Pt33`: 位置协方差矩阵上三角

## 可视化功能

### RViz可视化
OpenVINS提供了丰富的可视化功能，包括：
- **轨迹显示**: 实时显示估计的轨迹路径
- **特征点**: 显示当前跟踪的特征点
- **SLAM点云**: 显示3D地图点
- **相机姿态**: 显示相机的位置和朝向
- **协方差**: 显示位姿估计的不确定性

### 可视化控制
```bash
# 启用可视化（默认）
roslaunch ov_msckf cerberus.launch doviz:=true

# 关闭可视化（适合无头服务器或批量处理）
roslaunch ov_msckf cerberus.launch doviz:=false

# 使用自定义RViz配置
roslaunch ov_msckf cerberus.launch rviz_config:=/path/to/custom.rviz
```

### 可视化话题
OpenVINS发布以下可视化话题：
- `/ov_msckf/poseimu`: 位姿估计
- `/ov_msckf/pathimu`: 轨迹路径
- `/ov_msckf/trackhist`: 特征跟踪历史
- `/ov_msckf/points_msckf`: MSCKF特征点
- `/ov_msckf/points_slam`: SLAM特征点

## 注意事项

1. 确保bag文件路径正确
2. 确保有足够的磁盘空间保存结果
3. 运行前检查ROS环境是否正确设置
4. 如果某个序列失败，检查bag文件是否存在且格式正确
5. **可视化注意事项**:
   - 批量运行时建议关闭可视化（`doviz:=false`）
   - 可视化需要图形界面支持
   - RViz窗口可以通过鼠标交互调整视角

## 故障排除

### 常见问题
1. **Bag文件不存在**: 检查路径是否正确
2. **权限问题**: 确保有写入`/ws/results/`目录的权限
3. **ROS环境问题**: 确保已source OpenVINS的setup.bash

### 检查运行状态
```bash
# 检查轨迹文件是否生成
ls -la /ws/results/CERBERUS/*/seq*/trajectory.txt

# 查看轨迹文件内容
head -5 /ws/results/CERBERUS/stereo+imu/seq0/trajectory.txt
```
