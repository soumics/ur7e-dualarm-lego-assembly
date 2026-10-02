import rclpy, numpy as np, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); n = Node("grab"); b = CvBridge(); got = {}
def cb(k):
    def f(m): got.setdefault(k, m)
    return f
n.create_subscription(Image, "/camera/color/image_raw", cb("c"), 1)
n.create_subscription(Image, "/camera/depth/image_raw", cb("d"), 1)
while len(got) < 2: rclpy.spin_once(n, timeout_sec=1)
c = b.imgmsg_to_cv2(got["c"], "bgr8"); d = b.imgmsg_to_cv2(got["d"])
cv2.imwrite("/home/ros/ws/media/phase2_camera_color.png", c)
v = d[np.isfinite(d)]
print("depth enc", got["d"].encoding, "min %.3f max %.3f median %.3f" % (v.min(), v.max(), np.median(v)))
dn = np.clip((d - 0.80) / (1.10 - 0.80), 0, 1); dn[~np.isfinite(d)] = 1
cv2.imwrite("/home/ros/ws/media/phase2_camera_depth.png", cv2.applyColorMap((255 * (1 - dn)).astype(np.uint8), cv2.COLORMAP_JET))
