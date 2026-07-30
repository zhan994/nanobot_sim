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
body exclusion sphere are removed. An optional world-coordinate wire dropout
model can also remove returns from the power lines in ``uav_complex_120m``.

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


class WireDropoutModel:
    """Apply spatially and temporally correlated dropout to straight wires.

    The model is deliberately based on world-space geometry rather than point
    intensity: Gazebo Classic's Velodyne plugin treats ``min_intensity`` as a
    floor, so it is not a return-rejection threshold.
    """

    def __init__(
        self,
        wire_lines,
        x_min: float,
        x_max: float,
        detection_radius: float,
        segment_length: float,
        near_detection_probability: float,
        range_decay: float,
        temporal_persistence: float,
        segment_on_probability: float,
        hidden_probability_scale: float,
        angle_floor: float,
        pole_x,
        pole_exclusion_half_width: float,
        seed: int,
    ) -> None:
        lines = np.asarray(wire_lines, dtype=np.float64)
        if lines.ndim != 2 or lines.shape[1] != 2 or lines.shape[0] == 0:
            raise ValueError("~wire_lines must contain one or more [world_y, world_z] pairs")
        if not np.all(np.isfinite(lines)):
            raise ValueError("~wire_lines must contain only finite values")
        if not x_min < x_max:
            raise ValueError("~wire_x_min must be smaller than ~wire_x_max")
        if detection_radius <= 0.0:
            raise ValueError("~wire_detection_radius must be positive")
        if segment_length <= 0.0:
            raise ValueError("~wire_segment_length must be positive")
        if range_decay <= 0.0:
            raise ValueError("~wire_range_decay must be positive")
        for name, value in (
            ("~wire_near_detection_probability", near_detection_probability),
            ("~wire_temporal_persistence", temporal_persistence),
            ("~wire_segment_on_probability", segment_on_probability),
            ("~wire_hidden_probability_scale", hidden_probability_scale),
            ("~wire_angle_floor", angle_floor),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError("{} must be in [0, 1]".format(name))
        if pole_exclusion_half_width < 0.0:
            raise ValueError("~wire_pole_exclusion_half_width must be non-negative")

        poles = np.asarray(pole_x, dtype=np.float64)
        if poles.ndim != 1 or not np.all(np.isfinite(poles)):
            raise ValueError("~wire_pole_x must contain finite x coordinates")

        self.lines = lines
        self.x_min = float(x_min)
        self.x_max = float(x_max)
        self.radius_squared = float(detection_radius) ** 2
        self.segment_length = float(segment_length)
        self.near_probability = float(near_detection_probability)
        self.range_decay = float(range_decay)
        self.persistence = float(temporal_persistence)
        self.on_probability = float(segment_on_probability)
        self.hidden_scale = float(hidden_probability_scale)
        self.angle_floor = float(angle_floor)
        self.pole_x = poles
        self.pole_exclusion_half_width = float(pole_exclusion_half_width)
        self.rng = np.random.RandomState(int(seed))
        self.segment_states = {}

    def _update_segment_states(self, keys):
        """Return one correlated on/off state for every unique segment key."""
        states = {}
        off_to_on = (1.0 - self.persistence) * self.on_probability
        on_to_off = (1.0 - self.persistence) * (1.0 - self.on_probability)
        for key in keys:
            key_tuple = (int(key[0]), int(key[1]))
            if key_tuple in self.segment_states:
                previous = self.segment_states[key_tuple]
            else:
                previous = bool(self.rng.random_sample() < self.on_probability)
            transition_probability = on_to_off if previous else off_to_on
            current = (
                not previous
                if self.rng.random_sample() < transition_probability
                else previous
            )
            self.segment_states[key_tuple] = current
            states[key_tuple] = current
        return states

    def keep_mask(
        self,
        world_x: np.ndarray,
        world_y: np.ndarray,
        world_z: np.ndarray,
        valid: np.ndarray,
        sensor_origin: np.ndarray,
    ):
        """Return the updated valid mask and wire candidate/retained counts."""
        in_x_range = (world_x >= self.x_min) & (world_x <= self.x_max)

        # Pick the closest of the configured parallel power lines.
        distance_squared = np.stack(
            [
                (world_y - line_y) ** 2 + (world_z - line_z) ** 2
                for line_y, line_z in self.lines
            ],
            axis=0,
        )
        closest_line = np.argmin(distance_squared, axis=0)
        closest_distance_squared = np.take_along_axis(
            distance_squared, closest_line[np.newaxis, ...], axis=0
        )[0]
        candidates = valid & in_x_range & (closest_distance_squared <= self.radius_squared)

        # The ray return does not identify its collision object. Preserve points
        # close to poles so their tops are not mistaken for wire returns.
        if self.pole_x.size and self.pole_exclusion_half_width > 0.0:
            near_pole = np.zeros_like(candidates)
            for pole in self.pole_x:
                near_pole |= np.abs(world_x - pole) <= self.pole_exclusion_half_width
            candidates &= ~near_pole

        rows, columns = np.nonzero(candidates)
        candidate_count = len(rows)
        if candidate_count == 0:
            return valid, 0, 0

        candidate_x = world_x[rows, columns]
        candidate_y = world_y[rows, columns]
        candidate_z = world_z[rows, columns]
        candidate_lines = closest_line[rows, columns]
        segment_indices = np.floor(
            (candidate_x - self.x_min) / self.segment_length
        ).astype(np.int64)

        keys = np.column_stack((candidate_lines, segment_indices))
        unique_keys = np.unique(keys, axis=0)
        segment_states = self._update_segment_states(unique_keys)
        segment_scale = np.fromiter(
            (
                1.0
                if segment_states[(int(line), int(segment))]
                else self.hidden_scale
                for line, segment in keys
            ),
            dtype=np.float64,
            count=candidate_count,
        )

        delta_x = candidate_x - sensor_origin[0]
        delta_y = candidate_y - sensor_origin[1]
        delta_z = candidate_z - sensor_origin[2]
        ranges = np.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z)

        # The wire axis is world +X. Its apparent width approaches zero when a
        # beam travels parallel to that axis and is largest for a side-on hit.
        safe_ranges = np.maximum(ranges, 1.0e-9)
        direction_x = np.clip(delta_x / safe_ranges, -1.0, 1.0)
        side_on_factor = np.sqrt(np.maximum(0.0, 1.0 - direction_x * direction_x))
        angle_factor = self.angle_floor + (1.0 - self.angle_floor) * side_on_factor
        range_factor = np.exp(-ranges / self.range_decay)
        probability = np.clip(
            self.near_probability * range_factor * angle_factor * segment_scale,
            0.0,
            1.0,
        )

        retained = self.rng.random_sample(candidate_count) < probability
        updated = valid.copy()
        updated[rows[~retained], columns[~retained]] = False
        return updated, candidate_count, int(np.count_nonzero(retained))


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
        self.wire_dropout_enabled = bool(
            rospy.get_param("~wire_dropout_enabled", False)
        )
        self.wire_dropout = None
        self.last_wire_candidates = 0
        self.last_wire_retained = 0
        if self.wire_dropout_enabled:
            self.wire_dropout = WireDropoutModel(
                wire_lines=rospy.get_param(
                    "~wire_lines",
                    [[-18.1, 9.35], [-16.0, 9.2], [-13.9, 9.35]],
                ),
                x_min=float(rospy.get_param("~wire_x_min", -52.0)),
                x_max=float(rospy.get_param("~wire_x_max", 54.0)),
                detection_radius=float(
                    rospy.get_param("~wire_detection_radius", 0.10)
                ),
                segment_length=float(rospy.get_param("~wire_segment_length", 0.5)),
                near_detection_probability=float(
                    rospy.get_param("~wire_near_detection_probability", 0.72)
                ),
                range_decay=float(rospy.get_param("~wire_range_decay", 38.0)),
                temporal_persistence=float(
                    rospy.get_param("~wire_temporal_persistence", 0.90)
                ),
                segment_on_probability=float(
                    rospy.get_param("~wire_segment_on_probability", 0.65)
                ),
                hidden_probability_scale=float(
                    rospy.get_param("~wire_hidden_probability_scale", 0.12)
                ),
                angle_floor=float(rospy.get_param("~wire_angle_floor", 0.15)),
                pole_x=rospy.get_param(
                    "~wire_pole_x", [-52.0, -34.0, -16.0, 2.0, 20.0, 38.0, 54.0]
                ),
                pole_exclusion_half_width=float(
                    rospy.get_param("~wire_pole_exclusion_half_width", 0.35)
                ),
                seed=int(rospy.get_param("~wire_random_seed", 120)),
            )

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
            "body exclusion radius: %.2fm, wire dropout: %s",
            self.input_topic,
            self.output_topic,
            self.target_frame,
            mode,
            self.body_filter_radius,
            "enabled" if self.wire_dropout_enabled else "disabled",
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

        self.last_wire_candidates = 0
        self.last_wire_retained = 0
        if self.wire_dropout is not None:
            valid, self.last_wire_candidates, self.last_wire_retained = (
                self.wire_dropout.keep_mask(
                    output_x,
                    output_y,
                    output_z,
                    valid,
                    translation,
                )
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
                "pointcloud_to_body: published %d / %d points; wire returns %d / %d",
                output.width,
                message.width * message.height,
                self.last_wire_retained,
                self.last_wire_candidates,
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
