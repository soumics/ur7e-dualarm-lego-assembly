"""RViz view of what perception reports: /lego/detections -> /lego/detection_markers.

Each detected brick is drawn at real size (body + studs) in its detected colour, so RViz
shows exactly what the rest of the pipeline believes about the scene. Optionally a static
baseplate (not a detection; a known fixture of the workcell) is drawn too.

    ros2 run lego_perception detection_markers --ros-args -p baseplate_studs:=8 \
        -p baseplate_xyz:="[0.0, 0.0, 0.75]"
"""

import rclpy
from lego_msgs.msg import BrickDetectionArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

PITCH, HEIGHT, STUD_R, STUD_H, PLATE_H = 0.008, 0.0096, 0.0024, 0.0017, 0.0032
COLOURS = {   # same RGB as generate_bricks.py
    "red": (0.80, 0.05, 0.05), "blue": (0.05, 0.25, 0.80), "yellow": (0.95, 0.80, 0.05),
    "green": (0.10, 0.55, 0.15), "white": (0.92, 0.92, 0.92),
}


def _marker(frame, ns, mid, mtype, rgba):
    m = Marker()
    m.header.frame_id = frame
    m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.pose.orientation.w = 1.0
    return m


def stud_points(w, l, z):
    from geometry_msgs.msg import Point
    pts = []
    for i in range(l):
        for j in range(w):
            pts.append(Point(x=(i - (l - 1) / 2) * PITCH, y=(j - (w - 1) / 2) * PITCH, z=z))
    return pts


class DetectionMarkers(Node):
    def __init__(self):
        super().__init__("detection_markers")
        self.baseplate_studs = self.declare_parameter("baseplate_studs", 0).value
        self.baseplate_xyz = list(self.declare_parameter("baseplate_xyz", [0.0, 0.0, 0.75]).value)
        self.pub = self.create_publisher(MarkerArray, "/lego/detection_markers", 10)
        self.create_subscription(BrickDetectionArray, "/lego/detections", self.on_detections, 10)
        if self.baseplate_studs > 0:
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_publisher(MarkerArray, "/lego/fixture_markers", latched).publish(self.baseplate())

    def baseplate(self):
        n = self.baseplate_studs
        x, y, z = self.baseplate_xyz
        rgba = (0.35, 0.6, 0.35, 1.0)
        body = _marker("world", "baseplate", 0, Marker.CUBE, rgba)
        body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, z + PLATE_H / 2
        body.scale.x = body.scale.y = n * PITCH - 0.0002
        body.scale.z = PLATE_H
        # cylinders have no list type; small cubes read well at this scale
        studs = _marker("world", "baseplate", 1, Marker.CUBE_LIST, rgba)
        studs.pose.position.x, studs.pose.position.y, studs.pose.position.z = x, y, z
        studs.scale.x = studs.scale.y = 2 * STUD_R * 0.9
        studs.scale.z = STUD_H
        studs.points = stud_points(n, n, PLATE_H + STUD_H / 2)
        return MarkerArray(markers=[body, studs])

    def on_detections(self, msg):
        out = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        out.markers.append(clear)
        for k, d in enumerate(msg.detections):
            w, l = (int(v) for v in d.brick_type.split("x"))
            rgb = COLOURS.get(d.color, (0.5, 0.5, 0.5))
            body = _marker(msg.header.frame_id, "bricks", 2 * k, Marker.CUBE, (*rgb, 1.0))
            body.header.stamp = msg.header.stamp
            # pose is the bottom-face centre; the cube is centred on the body
            body.pose = _offset(d.pose, HEIGHT / 2)
            body.scale.x, body.scale.y, body.scale.z = l * PITCH - 0.0002, w * PITCH - 0.0002, HEIGHT
            studs = _marker(msg.header.frame_id, "studs", 2 * k + 1, Marker.CUBE_LIST,
                            tuple(min(1.0, c * 1.1) for c in rgb) + (1.0,))
            studs.header.stamp = msg.header.stamp
            studs.pose = d.pose
            studs.scale.x = studs.scale.y = 2 * STUD_R * 0.9
            studs.scale.z = STUD_H
            studs.points = stud_points(w, l, HEIGHT + STUD_H / 2)
            out.markers += [body, studs]
        self.pub.publish(out)


def _offset(pose, dz):
    """pose shifted by dz along its own z axis."""
    from geometry_msgs.msg import Pose
    q = pose.orientation
    # third column of the rotation matrix
    zx = 2 * (q.x * q.z + q.w * q.y)
    zy = 2 * (q.y * q.z - q.w * q.x)
    zz = 1 - 2 * (q.x * q.x + q.y * q.y)
    p = Pose()
    p.orientation = q
    p.position.x = pose.position.x + dz * zx
    p.position.y = pose.position.y + dz * zy
    p.position.z = pose.position.z + dz * zz
    return p


def main():
    rclpy.init()
    node = DetectionMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
