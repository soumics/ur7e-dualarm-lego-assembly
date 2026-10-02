import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
rclpy.init(); n = Node("cloudcheck"); buf = Buffer(); TransformListener(buf, n); got = []
n.create_subscription(PointCloud2, "/camera/points", lambda m: got.append(m), 1)
while not got: rclpy.spin_once(n, timeout_sec=1)
m = got[0]
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.2)
import sys; frame = sys.argv[1] if len(sys.argv) > 1 else m.header.frame_id
t = buf.lookup_transform("world", frame, rclpy.time.Time()).transform
rot = R.from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]); tr = np.array([t.translation.x, t.translation.y, t.translation.z])
p = pc2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)
w = rot.apply(p) + tr
on_table = w[(abs(w[:,0]) < 0.5) & (abs(w[:,1]) < 0.35)]
print("interpreted in", frame, "header", m.header.frame_id, "points", len(p))
print("world z of points over table area: median %.4f  (table top = 0.75)" % np.median(on_table[:,2]))
near = w[(abs(w[:,0]-0.05) < 0.012) & (abs(w[:,1]) < 0.006)]
print("points near brick centre: n=%d, max z %.4f (brick top ~0.7613)" % (len(near), near[:,2].max() if len(near) else -1))
tab = w[(w[:,2] > 0.70) & (w[:,2] < 0.80)]
print("table-height points: x range %.3f..%.3f  y range %.3f..%.3f (table x +/-0.6, y +/-0.4)" % (tab[:,0].min(), tab[:,0].max(), tab[:,1].min(), tab[:,1].max()))
hi = w[(w[:,2] > 0.764) & (w[:,2] < 0.80) & (abs(w[:,0]) < 0.45) & (abs(w[:,1]) < 0.3)]
print("raised points (brick?) n=%d centroid %s" % (len(hi), np.round(hi.mean(axis=0), 4) if len(hi) else None))
print("camera_info K:", list(np.round(np.array(cam.k), 2)) if 'cam' in dir() else "")
