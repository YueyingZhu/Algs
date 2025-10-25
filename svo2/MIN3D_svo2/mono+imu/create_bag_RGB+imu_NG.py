#!/usr/bin/env python3
import rospy
import rosbag
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import pandas as pd
import numpy as np
import os
from tqdm import tqdm

# === 修改路径 ===
acc_path = "~/Datasets/MIN3D/und_1/und_1_ngimu_acc.csv"
eul_path = "~/Datasets/MIN3D/und_1/und_1_ngimu_eul.csv"
img_dir  = "~/Datasets/MIN3D/und_1/RGB_RS"
output_bag = "/home/zyy/Datasets/MIN3D_svo2/RGB+imu/und_1.bag"

acc_path = os.path.expanduser(acc_path)
eul_path = os.path.expanduser(eul_path)
img_dir = os.path.expanduser(img_dir)

# === 读取数据 ===
df_acc = pd.read_csv(acc_path)
df_eul = pd.read_csv(eul_path)

# 清洗字段名
df_acc.columns = [c.strip() for c in df_acc.columns]
df_eul.columns = [c.strip() for c in df_eul.columns]

# 按 timestamp 合并数据
df_all = pd.merge_asof(df_acc.sort_values("timestamp"),
                       df_eul.sort_values("timestamp"),
                       on="timestamp")

# === 欧拉角差分计算角速度 ===
df_all["roll_vel"] = np.gradient(df_all["roll"], df_all["timestamp"])
df_all["pitch_vel"] = np.gradient(df_all["pitch"], df_all["timestamp"])
df_all["yaw_vel"] = np.gradient(df_all["yaw"], df_all["timestamp"])

# === 图像加载 ===
image_files = sorted([f for f in os.listdir(img_dir) if f.endswith(".png")])
bridge = CvBridge()

# === 写入 rosbag ===
bag = rosbag.Bag(output_bag, 'w')
print(f"Writing to {output_bag} ...")

try:
    for i, row in tqdm(df_all.iterrows(), total=len(df_all)):
        t = rospy.Time.from_sec(float(row["timestamp"]))

        # 构造 IMU 消息
        imu = Imu()
        imu.header = Header()
        imu.header.stamp = t
        imu.header.frame_id = "imu0"
        imu.linear_acceleration.x = row["earth_acceleration_x"]
        imu.linear_acceleration.y = row["earth_acceleration_y"]
        imu.linear_acceleration.z = row["earth_acceleration_z"]
        imu.angular_velocity.x = row["roll_vel"]
        imu.angular_velocity.y = row["pitch_vel"]
        imu.angular_velocity.z = row["yaw_vel"]

        bag.write("/imu0", imu, t)

    for img_name in tqdm(image_files):
        ts_float = float(os.path.splitext(img_name)[0])
        t = rospy.Time.from_sec(ts_float)
        img_path = os.path.join(img_dir, img_name)
        img_cv = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)

        if img_cv is None:
            continue

        img_msg = bridge.cv2_to_imgmsg(img_cv, encoding="passthrough")
        img_msg.header = Header()
        img_msg.header.stamp = t
        img_msg.header.frame_id = "cam0"

        bag.write("/cam0/image_raw", img_msg, t)

finally:
    bag.close()
    print(f"\nDone. Bag saved as: {output_bag}")

print("IMU时间范围：", df_all["timestamp"].min(), df_all["timestamp"].max())

img_ts = [float(f[:-4]) for f in image_files]  # assuming .png
print("图像时间范围：", min(img_ts), max(img_ts))

# 滚动平均平滑欧拉角（去掉微小抖动）
df_all["roll"]  = df_all["roll"].rolling(5, center=True, min_periods=1).mean()
df_all["pitch"] = df_all["pitch"].rolling(5, center=True, min_periods=1).mean()
df_all["yaw"]   = df_all["yaw"].rolling(5, center=True, min_periods=1).mean()

# 差分估算角速度
df_all["roll_vel"]  = np.gradient(df_all["roll"], df_all["timestamp"])
df_all["pitch_vel"] = np.gradient(df_all["pitch"], df_all["timestamp"])
df_all["yaw_vel"]   = np.gradient(df_all["yaw"], df_all["timestamp"])

# 限制角速度范围
df_all["roll_vel"]  = df_all["roll_vel"].clip(-3, 3)
df_all["pitch_vel"] = df_all["pitch_vel"].clip(-3, 3)
df_all["yaw_vel"]   = df_all["yaw_vel"].clip(-3, 3)

print("\n角速度估算范围：")
print(df_all[["roll_vel", "pitch_vel", "yaw_vel"]].describe())

