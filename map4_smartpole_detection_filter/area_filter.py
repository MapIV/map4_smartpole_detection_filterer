from pathlib import Path
from typing import List, Tuple

import numpy as np
import rclpy
import rclpy.time
from autoware_perception_msgs.msg import DetectedObjects
from geometry_msgs.msg import Point, TransformStamped
from matplotlib.path import Path as MplPath
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from transforms3d.euler import euler2mat
from visualization_msgs.msg import Marker, MarkerArray

from .area_filter_model import AreaFilterRootSchema, Type

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
        self.target_frame = (
            self.declare_parameter("target_frame", "base_link")
            .get_parameter_value()
            .string_value
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

        self.sub_objects = self.create_subscription(
            DetectedObjects,
            "input/objects",
            self.callback_objects,
            best_effort_profile if sub_qos == "best_effort" else reliable_profile,
        )
        self.pub_objects = self.create_publisher(
            DetectedObjects,
            "output/objects",
            best_effort_profile if pub_qos == "best_effort" else reliable_profile,
        )
        self.pub_markers = self.create_publisher(
            MarkerArray,
            "output/markers",
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )

        with config_path.open() as fp:
            self.config = AreaFilterRootSchema.model_validate_json(fp.read())

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        time_now = self.get_clock().now()

        markers = MarkerArray()
        self.paths: List[Tuple[MplPath, Type]] = []
        for i, data in enumerate(self.config.data):
            polygons = (
                np.array(data.polygons)
                - np.array([90918.756000, 95035.654000, 157.450000])
            ) @ euler2mat(0.029, -0.019, -0.041)

            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = time_now.to_msg()
            marker.ns = f"area_filter-{i:03d}-{data.type}"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0 if data.type == "white" else 0.0
            marker.color.g = 0.0 if data.type == "white" else 1.0
            marker.color.b = 0.0 if data.type == "white" else 1.0
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in polygons]
            markers.markers.append(marker)

            self.paths.append((MplPath(polygons[:, :2]), data.type))

        self.pub_markers.publish(markers)

    def callback_objects(self, in_msg: DetectedObjects):
        out_msg = DetectedObjects()
        out_msg.header = in_msg.header

        in_points = np.array(
            [
                [
                    in_object.kinematics.pose_with_covariance.pose.position.x,
                    in_object.kinematics.pose_with_covariance.pose.position.y,
                ]
                for in_object in in_msg.objects
            ]
        )
        should_remain = []
        for path, type in self.paths:
            contains_points = path.contains_points(in_points)
            should_remain.append(
                contains_points if type == Type.white else ~contains_points
            )

        should_remain = np.logical_and.reduce(should_remain)

        for i, in_object in enumerate(in_msg.objects):
            if should_remain[i]:
                out_msg.objects.append(in_object)

        self.pub_objects.publish(out_msg)


def main():
    rclpy.init()
    node = AreaFilter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
