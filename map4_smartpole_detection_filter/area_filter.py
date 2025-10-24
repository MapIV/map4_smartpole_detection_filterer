import colorsys
from pathlib import Path
from typing import List

import numpy as np
import rclpy
from autoware_perception_msgs.msg import TrackedObject, TrackedObjects
from geometry_msgs.msg import Point
from matplotlib.path import Path as MplPath
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray


class AreaFilter(Node):
    def __init__(self):
        super().__init__("area_filter")

        self.get_logger().set_level(LoggingSeverity.DEBUG)

        sub_qos = (
            self.declare_parameter("qos/sub", "reliable")
            .get_parameter_value()
            .string_value
        )
        pub_qos = (
            self.declare_parameter("qos/pub", "reliable")
            .get_parameter_value()
            .string_value
        )
        config_path = Path(
            self.declare_parameter("config_path", "").get_parameter_value().string_value
        )

        best_effort_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        if config_path.exists():
            self.sub_objects = self.create_subscription(
                TrackedObjects,
                "input/objects",
                self.callback,
                best_effort_profile if sub_qos == "best_effort" else reliable_profile,
            )
            self.pub_objects = self.create_publisher(
                TrackedObjects,
                "output/objects",
                best_effort_profile if pub_qos == "best_effort" else reliable_profile,
            )
            self.pub_markers = self.create_publisher(
                MarkerArray,
                "output/markers",
                QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
            )

            markers = MarkerArray()

            golden_ratio_conjugate = 0.61803398875

            self.white_paths: List[MplPath] = []
            count = 0
            for path in config_path.glob("*.white.txt"):
                self.get_logger().info(f"Loading whitelist: {path}")
                with path.open() as fp:
                    points = []
                    polygon = []
                    z_values = []
                    for line in fp:
                        x, y, z = tuple(map(float, line.strip().split(",")))
                        points.append(Point(x=x, y=y, z=z))
                        polygon.append((x, y))
                        z_values.append(z)

                    hue = (count * golden_ratio_conjugate) % 1.0
                    saturation = 1.0
                    value = 1.0
                    r, g, b = colorsys.hsv_to_rgb(hue, saturation, value)

                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = f"area_filter-{count:04d}-white-polygon"
                    marker.id = count
                    marker.type = Marker.LINE_STRIP
                    marker.action = Marker.ADD
                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = 0.1
                    marker.color.a = 1.0
                    marker.color.r = float(r)
                    marker.color.g = float(g)
                    marker.color.b = float(b)
                    marker.points = points
                    markers.markers.append(marker)

                    text_marker = Marker()
                    text_marker.header.frame_id = "map"
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.ns = f"area_filter-{count:04d}-white-text"
                    text_marker.id = count
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    text_marker.scale.z = 1.0
                    text_marker.color.a = 1.0
                    text_marker.color.r = float(r)
                    text_marker.color.g = float(g)
                    text_marker.color.b = float(b)
                    text_marker.text = path.name

                    polygon_np = np.array(polygon)
                    centroid = np.mean(polygon_np, axis=0)
                    mean_z = np.mean(z_values)
                    text_marker.pose.position.x = centroid[0]
                    text_marker.pose.position.y = centroid[1]
                    text_marker.pose.position.z = mean_z + 1.0
                    text_marker.pose.orientation.w = 1.0

                    markers.markers.append(text_marker)

                    self.white_paths.append(MplPath(polygon_np))
                    count += 1

            self.black_paths: List[MplPath] = []
            for path in config_path.glob("*.black.txt"):
                self.get_logger().info(f"Loading blacklist: {path}")
                with path.open() as fp:
                    points = []
                    polygon = []
                    z_values = []
                    for line in fp:
                        x, y, z = tuple(map(float, line.strip().split(",")))
                        points.append(Point(x=x, y=y, z=z))
                        polygon.append((x, y))
                        z_values.append(z)

                    hue = (count * golden_ratio_conjugate) % 1.0
                    saturation = 1.0
                    value = 1.0
                    r, g, b = colorsys.hsv_to_rgb(hue, saturation, value)

                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = f"area_filter-{count:04d}-black-polygon"
                    marker.id = count
                    marker.type = Marker.LINE_STRIP
                    marker.action = Marker.ADD
                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = 0.1
                    marker.color.a = 1.0
                    marker.color.r = float(r)
                    marker.color.g = float(g)
                    marker.color.b = float(b)
                    marker.points = points
                    markers.markers.append(marker)

                    text_marker = Marker()
                    text_marker.header.frame_id = "map"
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.ns = f"area_filter-{count:04d}-black-text"
                    text_marker.id = count
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    text_marker.scale.z = 1.0
                    text_marker.color.a = 1.0
                    text_marker.color.r = float(r)
                    text_marker.color.g = float(g)
                    text_marker.color.b = float(b)
                    text_marker.text = path.name

                    polygon_np = np.array(polygon)
                    centroid = np.mean(polygon_np, axis=0)
                    mean_z = np.mean(z_values)
                    text_marker.pose.position.x = centroid[0]
                    text_marker.pose.position.y = centroid[1]
                    text_marker.pose.position.z = mean_z + 1.0
                    text_marker.pose.orientation.w = 1.0

                    markers.markers.append(text_marker)

                    self.black_paths.append(MplPath(polygon_np))
                    count += 1

            self.pub_markers.publish(markers)

    def callback(self, in_msg: TrackedObjects):
        out_msg = TrackedObjects()
        out_msg.header = in_msg.header

        object: TrackedObject
        for object in in_msg.objects:
            point = [
                object.kinematics.pose_with_covariance.pose.position.x,
                object.kinematics.pose_with_covariance.pose.position.y,
            ]

            found_black = False
            for path in self.black_paths:
                if path.contains_point(point):
                    found_black = True
                    break

            if found_black:
                continue

            if self.white_paths:
                for path in self.white_paths:
                    if path.contains_point(point):
                        out_msg.objects.append(object)
            else:
                out_msg.objects.append(object)

        self.pub_objects.publish(out_msg)


def main():
    rclpy.init()
    node = AreaFilter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
