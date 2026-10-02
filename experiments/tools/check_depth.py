import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node("dc"); buf = Buffer(); TransformListener(buf, n); got = []
n.create_subscription(Image, "/camera/depth/image_raw", lambda m: got.append(m), 5)
while len(got) < 5: rclpy.spin_once(n, timeout_sec=1)
for _ in range(10): rclpy.spin_once(n, timeout_sec=0.2)
t = buf.lookup_transform("world", "camera_color_optical_frame", rclpy.time.Time()).transform.translation
d = np.stack([CvBridge().imgmsg_to_cv2(m) for m in got])
c = d[:, 200:280, 280:360]   # central patch, plain table
print("TF lens z %.4f -> expected table depth %.4f" % (t.z, t.z - 0.75))
print("depth centre patch over 5 frames: mean %.4f median %.4f std %.4f" % (np.nanmean(c), np.nanmedian(c), np.nanstd(c)))
