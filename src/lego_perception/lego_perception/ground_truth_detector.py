"""Ground-truth perception backend.

Reads model poses straight from Gazebo (gz-transport, /world/<world>/pose/info,
which carries model names) and publishes the bricks the overhead camera can see
as lego_msgs/BrickDetectionArray on /lego/detections - the same interface an
RGB-D detector will publish later.

Bricks are recognised by model name: lego_<type>_<color>_<index>, e.g. lego_2x4_red_0.
"Can see" = the brick centre projects inside the camera image (camera_info +
TF to the optical frame). Occlusion is not modelled.
"""

import re
import threading

import rclpy
from geometry_msgs.msg import Pose, PointStamped
from lego_msgs.msg import BrickDetection, BrickDetectionArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transforms)
from tf2_ros import Buffer, TransformException, TransformListener

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

BRICK_NAME = re.compile(r"^lego_(?P<type>\d+x\d+)_(?P<color>[a-z]+)_(?P<idx>\d+)$")
BRICK_HALF_HEIGHT = 0.0048  # project the body centre, not the bottom face


class GroundTruthDetector(Node):
    def __init__(self):
        super().__init__("ground_truth_detector")
        self.world = self.declare_parameter("world_name", "lego_world").value
        self.frame = self.declare_parameter("frame_id", "world").value
        self.camera_frame = self.declare_parameter(
            "camera_frame", "camera_color_optical_frame").value
        self.require_in_view = self.declare_parameter("require_in_view", True).value
        rate = self.declare_parameter("rate", 10.0).value

        self.lock = threading.Lock()
        self.poses = {}  # model name -> geometry_msgs/Pose
        self.camera_info = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_camera_info, 1)
        self.pub = self.create_publisher(BrickDetectionArray, "/lego/detections", 10)

        self.gz = GzNode()
        topic = f"/world/{self.world}/pose/info"
        if not self.gz.subscribe(Pose_V, topic, self.on_gz_poses):
            raise RuntimeError(f"could not subscribe to gz topic {topic}")
        self.create_timer(1.0 / rate, self.publish)
        self.get_logger().info(f"ground-truth detector on {topic}, in_view filter={self.require_in_view}")

    def on_camera_info(self, msg):
        self.camera_info = msg

    def on_gz_poses(self, msg):
        poses = {}
        for p in msg.pose:
            if BRICK_NAME.match(p.name):
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = p.position.x, p.position.y, p.position.z
                pose.orientation.x, pose.orientation.y = p.orientation.x, p.orientation.y
                pose.orientation.z, pose.orientation.w = p.orientation.z, p.orientation.w
                poses[p.name] = pose
        with self.lock:
            self.poses = poses

    def in_view(self, pose):
        info = self.camera_info
        if info is None:
            return False
        pt = PointStamped()
        pt.header.frame_id = self.frame
        pt.point.x, pt.point.y = pose.position.x, pose.position.y
        pt.point.z = pose.position.z + BRICK_HALF_HEIGHT
        try:
            c = self.tf_buffer.transform(pt, self.camera_frame).point
        except TransformException:
            return False
        if c.z <= 0.0:
            return False
        fx, cx, fy, cy = info.k[0], info.k[2], info.k[4], info.k[5]
        u, v = fx * c.x / c.z + cx, fy * c.y / c.z + cy
        return 0.0 <= u < info.width and 0.0 <= v < info.height

    def publish(self):
        with self.lock:
            poses = dict(self.poses)
        out = BrickDetectionArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame
        out.source = "ground_truth"
        for name, pose in sorted(poses.items()):
            if self.require_in_view and not self.in_view(pose):
                continue
            m = BRICK_NAME.match(name)
            d = BrickDetection()
            d.id = name
            d.brick_type = m.group("type")
            d.color = m.group("color")
            d.pose = pose
            d.confidence = 1.0
            out.detections.append(d)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GroundTruthDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
