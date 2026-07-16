#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PidController:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.previous_error = None

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        derivative = 0.0
        if self.previous_error is not None:
            derivative = (error - self.previous_error) / dt
        self.previous_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(output, self.output_min, self.output_max)


class StraightLineController:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.target_distance = rospy.get_param("~target_distance", 5.0)
        self.cruise_speed = rospy.get_param("~cruise_speed", 0.5)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.0)
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.03)
        self.control_rate = rospy.get_param("~control_rate", 20.0)
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 0.5)

        if self.target_distance <= 0.0:
            raise rospy.ROSInitException("~target_distance must be positive")
        if self.cruise_speed <= 0.0:
            raise rospy.ROSInitException("~cruise_speed must be positive")
        if self.control_rate <= 0.0:
            raise rospy.ROSInitException("~control_rate must be positive")
        if self.lookahead_distance <= 0.0:
            raise rospy.ROSInitException("~lookahead_distance must be positive")

        self.distance_pid = PidController(
            rospy.get_param("~distance_kp", 1.0),
            rospy.get_param("~distance_ki", 0.0),
            rospy.get_param("~distance_kd", 0.05),
            0.0,
            self.cruise_speed,
        )
        self.steering_pid = PidController(
            rospy.get_param("~steering_kp", 2.5),
            rospy.get_param("~steering_ki", 0.0),
            rospy.get_param("~steering_kd", 0.15),
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        self.start_pose = None
        self.finished = False
        self.latest_odom = None
        self.publisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.subscriber = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control)
        rospy.on_shutdown(self.stop)

    def odom_callback(self, msg):
        self.latest_odom = msg
        if self.start_pose is None:
            pose = msg.pose.pose
            self.start_pose = (
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            )
            rospy.loginfo(
                "Straight controller started at (%.3f, %.3f), yaw %.3f rad",
                self.start_pose[0],
                self.start_pose[1],
                self.start_pose[2],
            )

    def control(self, event):
        if self.latest_odom is None or self.start_pose is None:
            rospy.logwarn_throttle(2.0, "Waiting for odometry on %s", self.odom_topic)
            return

        if self.finished:
            self.publisher.publish(Twist())
            return

        pose = self.latest_odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = yaw_from_quaternion(pose.orientation)
        start_x, start_y, start_yaw = self.start_pose

        dx = x - start_x
        dy = y - start_y
        along_track = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
        cross_track = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
        remaining = self.target_distance - along_track

        if remaining <= self.distance_tolerance:
            self.finished = True
            self.publisher.publish(Twist())
            rospy.loginfo(
                "Target reached: along=%.3f m, lateral error=%.3f m",
                along_track,
                cross_track,
            )
            return

        dt = (event.current_real - event.last_real).to_sec()
        linear_speed = self.distance_pid.update(remaining, dt)
        line_correction = math.atan2(cross_track, self.lookahead_distance)
        steering_error = wrap_angle(start_yaw - yaw - line_correction)

        command = Twist()
        command.linear.x = linear_speed
        command.angular.z = self.steering_pid.update(steering_error, dt)
        self.publisher.publish(command)

        rospy.loginfo_throttle(
            1.0,
            "along %.2f/%.2f m, lateral %.3f m, heading error %.3f rad",
            along_track,
            self.target_distance,
            cross_track,
            steering_error,
        )

    def stop(self):
        self.publisher.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("straight_line_controller")
    try:
        StraightLineController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
