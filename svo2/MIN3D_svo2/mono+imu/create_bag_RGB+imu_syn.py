import os
import cv2
import csv
import rospy
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header

# === 用户输入 und_i 序号 ===
seq = input("请输入 und 序号（如 1/2/3/4/5）: ").strip()
if not seq.isdigit():
    print("❌ 无效输入：请输入数字")
    exit(1)

# === 动态路径配置 ===
rgb_dir = os.path.expanduser(f'~/Datasets/MIN3D/und_{seq}/RGB_RS')
imu_csv = os.path.expanduser(f'~/Datasets/MIN3D/und_{seq}/und_{seq}_imu_rs.csv')
output_bag = os.path.expanduser(f'~/Datasets/MIN3D_svo2/RGB+imu/und_{seq}.bag')

# === 检查文件存在性 ===
if not os.path.exists(rgb_dir):
    print(f"❌ 图像路径不存在: {rgb_dir}")
    exit(1)
if not os.path.exists(imu_csv):
    print(f"❌ IMU 文件不存在: {imu_csv}")
    exit(1)

# === 加载图像文件名与时间戳 ===
image_files = sorted([f for f in os.listdir(rgb_dir) if f.endswith('.png')])
image_times = [float(f.replace('.png', '')) for f in image_files]
t_img0 = min(image_times)

# === 加载 IMU 数据与时间戳 ===
imu_times = []
imu_data = []
with open(imu_csv) as f:
    reader = csv.reader(f)
    next(reader)
    for row in reader:
        try:
            t = float(row[0])
            imu_times.append(t)
            imu_data.append(row)
        except:
            continue
t_imu0 = min(imu_times)

# === 初始化 ROS Bag 写入 ===
bag = rosbag.Bag(output_bag, 'w')
bridge = CvBridge()

# === 写入图像消息 ===
for fname in image_files:
    try:
        t = float(fname.replace('.png', '')) - t_img0 + 1e-6
        stamp = rospy.Time.from_sec(t)
        img_path = os.path.join(rgb_dir, fname)
        img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            continue
        encoding = "mono8" if len(img.shape) == 2 else "rgb8"
        msg = bridge.cv2_to_imgmsg(img, encoding=encoding)
        msg.header = Header(stamp=stamp, frame_id="cam0")
        bag.write('/cam0/image_raw', msg, stamp)
    except Exception as e:
        print(f"[Image WARN] {fname}: {e}")
        continue

# === 写入 IMU 消息 ===
for row in imu_data:
    try:
        t = float(row[0]) - t_imu0 + 1e-6
        stamp = rospy.Time.from_sec(t)
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = "imu0"
        imu_msg.linear_acceleration.x = float(row[1])
        imu_msg.linear_acceleration.y = float(row[2])
        imu_msg.linear_acceleration.z = float(row[3])
        imu_msg.angular_velocity.x = float(row[4])
        imu_msg.angular_velocity.y = float(row[5])
        imu_msg.angular_velocity.z = float(row[6])
        bag.write('/imu0', imu_msg, stamp)
    except Exception as e:
        print(f"[IMU WARN] {row[0]}: {e}")
        continue

bag.close()
print(f"[DONE] Bag saved to {output_bag}")

