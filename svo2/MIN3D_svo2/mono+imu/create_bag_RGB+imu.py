import os
import sys
import cv2
import csv
import rospy
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header

if len(sys.argv) != 2:
    print("❌ 用法: python3 create_bag_RGB+imu.py <und序号>")
    sys.exit(1)

seq = sys.argv[1]
rgb_dir = os.path.expanduser(f'~/Datasets/MIN3D/und_{seq}/RGB_RS')
imu_csv = os.path.expanduser(f'~/Datasets/MIN3D/und_{seq}/und_{seq}_imu_rs.csv')
output_bag = os.path.expanduser(f'~/Datasets/MIN3D_svo2/RGB+imu/und_{seq}.bag')

if not os.path.exists(rgb_dir):
    print(f"❌ 图像路径不存在: {rgb_dir}")
    sys.exit(1)
if not os.path.exists(imu_csv):
    print(f"❌ IMU 文件不存在: {imu_csv}")
    sys.exit(1)

bridge = CvBridge()
bag = rosbag.Bag(output_bag, 'w')

# === 图像部分 ===
image_files = sorted([f for f in os.listdir(rgb_dir) if f.endswith('.png')])
print(f"[INFO] Found {len(image_files)} images")

for fname in image_files:
    try:
        t = float(fname.replace('.png', ''))  # e.g., 1668016988.707888
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
        print(f"[WARN] Failed on image {fname}: {e}")
        continue

print(f"[INFO] 图像打包完成")

# === IMU部分 ===
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
            imu_msg.angular_velocity.x = float(row[1])
            imu_msg.angular_velocity.y = float(row[2])
            imu_msg.angular_velocity.z = float(row[3])
            imu_msg.linear_acceleration.x = float(row[4])
            imu_msg.linear_acceleration.y = float(row[5])
            imu_msg.linear_acceleration.z = float(row[6])
            bag.write('/imu0', imu_msg, stamp)
        except Exception as e:
            print(f"[WARN] Failed on IMU row: {e}")
            continue

print(f"[INFO] IMU打包完成")
bag.close()
print(f"[✅ DONE] Bag saved to {output_bag}")

bag.close()
print(f"[✅ DONE] Bag saved to {output_bag}")

