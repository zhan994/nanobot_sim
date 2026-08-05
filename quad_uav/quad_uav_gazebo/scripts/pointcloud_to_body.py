#!/usr/bin/env python3
"""Transform a ROS 1 PointCloud2 topic from the lidar frame to the world frame.

The default static extrinsic matches the SDF supplied with this script:

    lidar pose in base_link = (0.10, 0.0, 0.15), RPY = (0, pi/4, 0)

In the default mode the transform is composed as ``world_T_lidar =
world_T_body * body_T_lidar``, where ``world_T_body`` comes from MAVROS
odometry.  This is the coordinate system expected by Diff-Planner's lidar
mapping path: it reads point coordinates directly and does not transform them
according to ``PointCloud2.header.frame_id``.

All PointCloud2 fields (for example, intensity and ring) and the timestamp
are preserved. The output is unorganized because points inside the configurable
body exclusion sphere are removed.

Example:
    rosrun <your_package> pointcloud_to_body.py \
      _input_topic:=/velodyne_points \
      _output_topic:=/velodyne_points_world \
      _odom_topic:=/mavros/local_position/odom \
      _body_filter_radius:=1.0 \
      _target_frame:=world

Set ``_use_tf:=true`` only if TF already provides a direct transform from the
input cloud frame to ``target_frame`` at the cloud timestamp. In that mode,
the static SDF extrinsic and odometry parameters are ignored.
"""

import math
from typing import Iterable, Tuple

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField


def _three_values(value: Iterable[float], parameter: str) -> Tuple[float, float, float]:
    values = tuple(float(item) for item in value)
    if len(values) != 3:
        raise ValueError("{} must contain exactly three values".format(parameter))
    return values


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the rotation matrix Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm < 1.0e-24:
        raise ValueError("received a zero-length transform quaternion")

    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale

    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float64,
    )


class PointCloudToBody:
    def __init__(self) -> None:
        self.input_topic = rospy.get_param("~input_topic", "/velodyne_points")
        self.output_topic = rospy.get_param("~output_topic", "/velodyne_points_world")
        self.target_frame = str(rospy.get_param("~target_frame", "world")).lstrip("/")
        self.body_frame = str(rospy.get_param("~body_frame", "base_link")).lstrip("/")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.max_odom_age = float(rospy.get_param("~max_odom_age", 0.10))
        self.body_filter_radius = float(rospy.get_param("~body_filter_radius", 1.0))
        if self.body_filter_radius < 0.0:
            raise ValueError("~body_filter_radius must be non-negative")
        self.use_tf = bool(rospy.get_param("~use_tf", False))

        translation = _three_values(
            rospy.get_param("~static_translation", [0.10, 0, 0.15]),
            "~static_translation",
        )
        rpy = _three_values(
            rospy.get_param("~static_rpy", [0.0, 0.785398, 0.0]),
            "~static_rpy",
        )
        self.static_translation = np.asarray(translation, dtype=np.float64)
        self.static_rotation = _rpy_matrix(*rpy)
        self.latest_odom = None

        self.tf_buffer = None
        self.tf_listener = None
        if self.use_tf:
            self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        else:
            self.odom_subscriber = rospy.Subscriber(
                self.odom_topic,
                Odometry,
                self._odom_callback,
                queue_size=10,
                tcp_nodelay=True,
            )

        self.publisher = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self._cloud_callback,
            queue_size=1,
            buff_size=64 * 1024 * 1024,
            tcp_nodelay=True,
        )

        mode = "direct TF" if self.use_tf else "static SDF extrinsic + odometry"
        rospy.loginfo(
            "pointcloud_to_body: %s -> %s, target frame '%s', mode: %s, "
            "body exclusion radius: %.2fm",
            self.input_topic,
            self.output_topic,
            self.target_frame,
            mode,
            self.body_filter_radius,
        )

    def _odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    @staticmethod
    def _xyz_fields(message: PointCloud2):
        fields = {field.name: field for field in message.fields}
        missing = [name for name in ("x", "y", "z") if name not in fields]
        if missing:
            raise ValueError("PointCloud2 is missing field(s): {}".format(", ".join(missing)))

        xyz = tuple(fields[name] for name in ("x", "y", "z"))
        datatypes = {field.datatype for field in xyz}
        if len(datatypes) != 1 or next(iter(datatypes)) not in (
            PointField.FLOAT32,
            PointField.FLOAT64,
        ):
            raise ValueError("x, y and z must all be FLOAT32 or all be FLOAT64")
        if any(field.count != 1 for field in xyz):
            raise ValueError("x, y and z fields must each have count=1")
        return xyz

    @staticmethod
    def _coordinate_view(data, message: PointCloud2, field: PointField) -> np.ndarray:
        kind = "f4" if field.datatype == PointField.FLOAT32 else "f8"
        byte_order = ">" if message.is_bigendian else "<"
        return np.ndarray(
            shape=(message.height, message.width),
            dtype=np.dtype(byte_order + kind),
            buffer=data,
            offset=field.offset,
            strides=(message.row_step, message.point_step),
        )

    def _get_transform(self, message: PointCloud2):
        if self.use_tf:
            source_frame = message.header.frame_id.lstrip("/")
            if not source_frame:
                raise ValueError("input cloud has an empty frame_id")

            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                message.header.stamp,
                rospy.Duration(0.05),
            ).transform
            q = transform.rotation
            t = transform.translation
            return _quaternion_matrix(q.x, q.y, q.z, q.w), np.array(
                [t.x, t.y, t.z], dtype=np.float64
            )

        if self.latest_odom is None:
            raise ValueError("no odometry received on {}".format(self.odom_topic))

        odom_stamp = self.latest_odom.header.stamp
        if not odom_stamp.is_zero() and not message.header.stamp.is_zero():
            odom_age = abs((message.header.stamp - odom_stamp).to_sec())
            if odom_age > self.max_odom_age:
                raise ValueError(
                    "odometry is {:.3f}s away from cloud timestamp (limit {:.3f}s)".format(
                        odom_age, self.max_odom_age
                    )
                )

        pose = self.latest_odom.pose.pose
        q = pose.orientation
        world_rotation = _quaternion_matrix(q.x, q.y, q.z, q.w)
        world_translation = np.array(
            [pose.position.x, pose.position.y, pose.position.z], dtype=np.float64
        )

        # world_T_lidar = world_T_body * body_T_lidar.
        return (
            world_rotation.dot(self.static_rotation),
            world_rotation.dot(self.static_translation) + world_translation,
        )

    def _get_body_transform(self, message: PointCloud2):
        """Return body_T_lidar for body-centred point filtering."""
        if not self.use_tf:
            return self.static_rotation, self.static_translation

        source_frame = message.header.frame_id.lstrip("/")
        if not source_frame:
            raise ValueError("input cloud has an empty frame_id")
        transform = self.tf_buffer.lookup_transform(
            self.body_frame,
            source_frame,
            message.header.stamp,
            rospy.Duration(0.05),
        ).transform
        q = transform.rotation
        t = transform.translation
        return _quaternion_matrix(q.x, q.y, q.z, q.w), np.array(
            [t.x, t.y, t.z], dtype=np.float64
        )

    def _transform_cloud(
        self,
        message: PointCloud2,
        rotation: np.ndarray,
        translation: np.ndarray,
        body_rotation: np.ndarray,
        body_translation: np.ndarray,
    ) -> PointCloud2:
        x_field, y_field, z_field = self._xyz_fields(message)

        # Copy the complete binary payload first so fields such as intensity and
        # ring remain byte-for-byte unchanged.
        output_data = bytearray(message.data)
        source_x = self._coordinate_view(message.data, message, x_field).astype(
            np.float64, copy=True
        )
        source_y = self._coordinate_view(message.data, message, y_field).astype(
            np.float64, copy=True
        )
        source_z = self._coordinate_view(message.data, message, z_field).astype(
            np.float64, copy=True
        )

        valid = np.isfinite(source_x) & np.isfinite(source_y) & np.isfinite(source_z)
        if self.body_filter_radius > 0.0:
            body_x = (
                body_rotation[0, 0] * source_x
                + body_rotation[0, 1] * source_y
                + body_rotation[0, 2] * source_z
                + body_translation[0]
            )
            body_y = (
                body_rotation[1, 0] * source_x
                + body_rotation[1, 1] * source_y
                + body_rotation[1, 2] * source_z
                + body_translation[1]
            )
            body_z = (
                body_rotation[2, 0] * source_x
                + body_rotation[2, 1] * source_y
                + body_rotation[2, 2] * source_z
                + body_translation[2]
            )
            valid &= (
                body_x * body_x + body_y * body_y + body_z * body_z
                >= self.body_filter_radius * self.body_filter_radius
            )

        output_x = self._coordinate_view(output_data, message, x_field)
        output_y = self._coordinate_view(output_data, message, y_field)
        output_z = self._coordinate_view(output_data, message, z_field)

        output_x[...] = (
            rotation[0, 0] * source_x
            + rotation[0, 1] * source_y
            + rotation[0, 2] * source_z
            + translation[0]
        )
        output_y[...] = (
            rotation[1, 0] * source_x
            + rotation[1, 1] * source_y
            + rotation[1, 2] * source_z
            + translation[1]
        )
        output_z[...] = (
            rotation[2, 0] * source_x
            + rotation[2, 1] * source_y
            + rotation[2, 2] * source_z
            + translation[2]
        )

        # Filtering individual points makes an organized cloud invalid. Pack
        # all retained point records into a compact, unorganized PointCloud2.
        filtered_data = bytearray()
        for row, column in zip(*np.nonzero(valid)):
            offset = row * message.row_step + column * message.point_step
            filtered_data.extend(output_data[offset : offset + message.point_step])

        output = PointCloud2()
        output.header.seq = message.header.seq
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.target_frame
        output.height = 1
        output.width = len(filtered_data) // message.point_step
        output.fields = message.fields
        output.is_bigendian = message.is_bigendian
        output.point_step = message.point_step
        output.row_step = output.width * message.point_step
        output.data = bytes(filtered_data)
        output.is_dense = True
        return output

    def _cloud_callback(self, message: PointCloud2) -> None:
        try:
            rotation, translation = self._get_transform(message)
            body_rotation, body_translation = self._get_body_transform(message)
            output = self._transform_cloud(
                message, rotation, translation, body_rotation, body_translation
            )
            self.publisher.publish(output)
            rospy.logdebug_throttle(
                2.0,
                "pointcloud_to_body: published %d / %d points after %.2fm body filter",
                output.width,
                message.width * message.height,
                self.body_filter_radius,
            )
        except (
            ValueError,
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            rospy.logwarn_throttle(2.0, "pointcloud_to_body: %s", str(error))


def main() -> None:
    rospy.init_node("pointcloud_to_body")
    try:
        PointCloudToBody()
    except (TypeError, ValueError) as error:
        rospy.logfatal("pointcloud_to_body configuration error: %s", str(error))
        raise SystemExit(2)
    rospy.spin()


if __name__ == "__main__":
    main()
