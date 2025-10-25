import os
import sys
import cv2
import rospy
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header

if len(sys.argv) != 2:
    print("❌ 用法: python3 create_bag_RGB_only.py <und序号>")
    sys.exit(1)

seq = sys.argv[1]
rgb_dir = os.path.expanduser(f'~/Datasets/MIN3D/und_{seq}/RGB_RS')
output_bag = os.path.expanduser(f'~/Datasets/MIN3D_svo2/RGB/und_{seq}.bag')

if not os.path.exists(rgb_dir):
    print(f"❌ 图像路径不存在: {rgb_dir}")
    sys.exit(1)

bridge = CvBridge()
bag = rosbag.Bag(output_bag, 'w')
image_ext = '.png'

image_files = sorted([f for f in os.listdir(rgb_dir) if f.endswith(image_ext)])
print(f"[INFO] Found {len(image_files)} images")

for fname in image_files:
    try:
        t = float(fname.replace('.png', ''))  # 秒级时间戳
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

bag.close()
print(f"[✅ DONE] Bag saved to {output_bag}")

