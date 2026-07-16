import colorsys
import math
import time
from pathlib import Path
from typing import List

import numpy as np
import rclpy
from autoware_perception_msgs.msg import TrackedObject, TrackedObjects
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
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

        # ASIL-A self-diagnostics. The area filter is OPT-IN: with no config the node still
        # subscribes and passes every object through, and reports that inactive state at OK
        # (INFO) -- it is an expected configuration, not a fault. _config_loaded is a diag field.
        self._config_loaded = config_path.exists()
        self._diag_msgs_in = 0
        self._diag_objects_in = 0
        self._diag_objects_out = 0
        self._diag_nonfinite = 0
        self._diag_last_input = time.monotonic()

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

        # Opt-in area filter: the subscription and publishers are ALWAYS created, so a pole
        # with no area config -- or an empty one -- passes every object through (see callback).
        # A missing/empty config is a valid "filter disabled" state, not a fault. Polygons are
        # loaded only when a config directory is present.
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
        self.black_paths: List[MplPath] = []

        if config_path.exists():
            count = 0
            for path in config_path.glob("*.white.txt"):
                self.get_logger().info(f"Loading whitelist: {path}")
                with path.open() as fp:
                    points = []
                    polygon = []
                    z_values = []
                    for line in fp:
                        stripped = line.strip()
                        if not stripped:
                            continue  # EDG-M8: skip blank lines
                        try:
                            x, y, z = tuple(map(float, stripped.split(",")))
                        except ValueError:
                            # EDG-M8: a malformed row must not raise during __init__ (that would
                            # leave the node dead on construction, with no diagnostics either).
                            self.get_logger().warning(
                                f"Skipping malformed polygon row in {path.name}: {stripped!r}"
                            )
                            continue
                        points.append(Point(x=x, y=y, z=z))
                        polygon.append((x, y))
                        z_values.append(z)

                    if len(polygon) < 3:
                        # EDG-M8: a degenerate polygon (<3 valid vertices) would build a
                        # meaningless MplPath; skip it rather than filter on garbage.
                        self.get_logger().warning(
                            f"Skipping polygon {path.name}: only {len(polygon)} valid vertices "
                            "(need >=3)"
                        )
                        continue

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

            for path in config_path.glob("*.black.txt"):
                self.get_logger().info(f"Loading blacklist: {path}")
                with path.open() as fp:
                    points = []
                    polygon = []
                    z_values = []
                    for line in fp:
                        stripped = line.strip()
                        if not stripped:
                            continue  # EDG-M8: skip blank lines
                        try:
                            x, y, z = tuple(map(float, stripped.split(",")))
                        except ValueError:
                            # EDG-M8: a malformed row must not raise during __init__ (that would
                            # leave the node dead on construction, with no diagnostics either).
                            self.get_logger().warning(
                                f"Skipping malformed polygon row in {path.name}: {stripped!r}"
                            )
                            continue
                        points.append(Point(x=x, y=y, z=z))
                        polygon.append((x, y))
                        z_values.append(z)

                    if len(polygon) < 3:
                        # EDG-M8: a degenerate polygon (<3 valid vertices) would build a
                        # meaningless MplPath; skip it rather than filter on garbage.
                        self.get_logger().warning(
                            f"Skipping polygon {path.name}: only {len(polygon)} valid vertices "
                            "(need >=3)"
                        )
                        continue

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

        # Report the opt-in state at INFO: active (with polygons) or pass-through (no config).
        if self.white_paths or self.black_paths:
            self.get_logger().info(
                f"Area filter active: {len(self.white_paths)} whitelist, "
                f"{len(self.black_paths)} blacklist polygon(s)."
            )
        else:
            self.get_logger().info(
                f"Area filter inactive (opt-in): no polygons at '{config_path}' -- "
                "passing all objects through."
            )

        # Always start diagnostics, even when config is missing (so the inactive-filter
        # state is reported rather than silently swallowed).
        # EDG-M9: diagnostic_updater.Updater already creates its OWN internal timer (period from
        # the diagnostic_updater.period param). A second explicit force_update() timer double-
        # published /diagnostics and left diagnostics.period_sec racy; pass the period in and let
        # the Updater's single internal timer drive updates.
        diag_period = float(self.declare_parameter("diagnostics.period_sec", 1.0).value)
        self._updater = Updater(self, diag_period)
        self._updater.setHardwareID(
            self.declare_parameter("diagnostics.hardware_id", "detection_filter").value
        )
        self._updater.add("area_filter", self._diag_task)

    def _diag_task(self, stat):
        age = time.monotonic() - self._diag_last_input
        n_white = len(getattr(self, "white_paths", []))
        n_black = len(getattr(self, "black_paths", []))
        # Opt-in filter: with no polygons (config absent or empty) the node passes every object
        # through by design -> OK (INFO), not a fault. Stall / non-finite checks only apply when
        # the filter is actually active (has polygons).
        if n_white == 0 and n_black == 0:
            level = DiagnosticStatus.OK
            message = "area filter inactive (no config): passing all objects through"
        elif self._diag_msgs_in == 0 and age > 5.0:
            level, message = DiagnosticStatus.ERROR, "No input objects received"
        elif age > 5.0:
            level, message = DiagnosticStatus.ERROR, "Input objects stalled"
        elif self._diag_nonfinite > 0:
            level = DiagnosticStatus.WARN
            message = "Non-finite object positions observed"
        else:
            level, message = DiagnosticStatus.OK, "Filtering nominally"
        stat.summary(level, message)
        stat.add("config_loaded", str(self._config_loaded))
        stat.add("whitelist_polygons", str(n_white))
        stat.add("blacklist_polygons", str(n_black))
        stat.add("messages_in_total", str(self._diag_msgs_in))
        stat.add("objects_in_total", str(self._diag_objects_in))
        stat.add("objects_out_total", str(self._diag_objects_out))
        stat.add("objects_filtered_total", str(self._diag_objects_in - self._diag_objects_out))
        stat.add("nonfinite_positions_total", str(self._diag_nonfinite))
        stat.add("input_age_sec", f"{age:.2f}")
        return stat

    def callback(self, in_msg: TrackedObjects):
        self._diag_last_input = time.monotonic()
        self._diag_msgs_in += 1
        self._diag_objects_in += len(in_msg.objects)

        out_msg = TrackedObjects()
        out_msg.header = in_msg.header

        object: TrackedObject
        for object in in_msg.objects:
            px = object.kinematics.pose_with_covariance.pose.position.x
            py = object.kinematics.pose_with_covariance.pose.position.y
            if not (math.isfinite(px) and math.isfinite(py)):
                self._diag_nonfinite += 1
                continue  # FSR-1.1: drop objects with non-finite position
            point = [px, py]

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
                        break  # EDG-M7: emit at most once even if inside several whitelist polygons
            else:
                out_msg.objects.append(object)

        self._diag_objects_out += len(out_msg.objects)
        self.pub_objects.publish(out_msg)


def main():
    rclpy.init()
    node = AreaFilter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
