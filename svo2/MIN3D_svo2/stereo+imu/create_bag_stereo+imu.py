import os
import cv2
import csv
import rospy
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header
from datetime import datetime

# === 配置 ===
stereo_left_dir = os.path.expanduser('~/Datasets/MIN3D/und_1/Stereo_L')
stereo_right_dir = os.path.expanduser('~/Datasets/MIN3D/und_1/Stereo_R')
imu_csv = os.path.expanduser('~/Datasets/MIN3D/und_1/und_1_imu_rs.csv')
output_bag = os.path.expanduser('~/Datasets/MIN3D_svo2/stereo+imu/und_1.bag')

# === 初始化 ===
bridge = CvBridge()
bag = rosbag.Bag(output_bag, 'w')
image_ext = '.png'

# === 读取图像时间戳 ===
left_imgs = sorted([f for f in os.listdir(stereo_left_dir) if f.endswith(image_ext)])
right_imgs = sorted([f for f in os.listdir(stereo_right_dir) if f.endswith(image_ext)])
common_imgs = sorted(list(set(left_imgs).intersection(set(right_imgs))))

print(f"[INFO] Total synced stereo frames: {len(common_imgs)}")

for fname in common_imgs:
    t = float(fname.replace(image_ext, ''))  # 秒级时间戳
    stamp = rospy.Time.from_sec(t)

    # 左图
    img_L = cv2.imread(os.path.join(stereo_left_dir, fname), cv2.IMREAD_UNCHANGED)
    if img_L is None:
        continue
    msg_L = bridge.cv2_to_imgmsg(img_L, encoding="mono8")
    msg_L.header = Header(stamp=stamp, frame_id="cam0")
    bag.write('/cam0/image_raw', msg_L, stamp)

    # 右图
    img_R = cv2.imread(os.path.join(stereo_right_dir, fname), cv2.IMREAD_UNCHANGED)
    if img_R is None:
        continue
    msg_R = bridge.cv2_to_imgmsg(img_R, encoding="mono8")
    msg_R.header = Header(stamp=stamp, frame_id="cam1")
    bag.write('/cam1/image_raw', msg_R, stamp)

print(f"[INFO] Stereo images written.")

# === 写入 IMU 数据 ===
with open(imu_csv) as f:
    reader = csv.reader(f)
    header = next(reader)
    for row in reader:
        try:
            t = float(row[0])
            stamp = rospy.Time.from_sec(t)

            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = "imu0"

            # 假设字段为：timestamp,w_RS_S_x,y,z,a_RS_S_x,y,z
            imu_msg.angular_velocity.x = float(row[1])
            imu_msg.angular_velocity.y = float(row[2])
            imu_msg.angular_velocity.z = float(row[3])
            imu_msg.linear_acceleration.x = float(row[4])
            imu_msg.linear_acceleration.y = float(row[5])
            imu_msg.linear_acceleration.z = float(row[6])

            bag.write('/imu0', imu_msg, stamp)
        except:
            continue

print(f"[INFO] IMU data written.")

bag.close()
print(f"[DONE] Bag saved to {output_bag}")

