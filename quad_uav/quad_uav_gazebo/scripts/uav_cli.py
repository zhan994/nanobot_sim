#!/usr/bin/env python3
"""Unified interactive CLI for simulated UAV RC, PD, and goal commands."""

import argparse
import math
import shlex
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCIn
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


DEFAULT_RC_CHANNELS = [
    1495,
    1495,
    1495,
    1495,
    1995,
    1995,
    1495,
    1495,
    1495,
    1945,
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Interactive UAV RC, PD-control, and PoseStamped goal publisher."
    )
    parser.add_argument("--rc-topic", default="/mavros/rc/in_sim")
    parser.add_argument("--cmd-topic", default="/setpoints_cmd")
    parser.add_argument("--odom-topic", default="/mavros/local_position/odom")
    parser.add_argument("--goal-topic", default="/move_base_simple/goal")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--kp", type=float, default=1.0)
    parser.add_argument("--kd", type=float, default=0.5)
    parser.add_argument("--max-vel", type=float, default=5.0)
    parser.add_argument("--reach-distance", type=float, default=0.5)
    parser.add_argument("--odom-timeout", type=float, default=1.0)
    parser.add_argument("--rc-rate", type=float, default=30.0)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def validate_args(args):
    positive_options = {
        "--max-vel": args.max_vel,
        "--reach-distance": args.reach_distance,
        "--odom-timeout": args.odom_timeout,
        "--rc-rate": args.rc_rate,
    }
    for name, value in positive_options.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("{} must be a positive number".format(name))
    for name, value in (("--kp", args.kp), ("--kd", args.kd)):
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("{} must be a non-negative number".format(name))


class UavCli:
    HOME_TARGET = (0.0, 0.0, 1.5, 0.0)
    TASK_TARGET = (0.5, 4.0, 1.0, 0.0)
    RC_PULSE_SECONDS = 2.0
    RC_PULSE_LOW = 1495
    RC_PULSE_HIGH = 1995

    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()

        self.rc_msg = RCIn()
        self.rc_msg.channels = list(DEFAULT_RC_CHANNELS)
        self.rc_pulse_deadline = None

        self.current_target = None
        self.current_pos = None
        self.current_vel = None
        self.last_odom_time = None

        self.rc_pub = rospy.Publisher(args.rc_topic, RCIn, queue_size=10)
        self.cmd_pub = rospy.Publisher(args.cmd_topic, PositionCommand, queue_size=10)
        self.goal_pub = rospy.Publisher(
            args.goal_topic, PoseStamped, queue_size=10
        )
        self.odom_sub = rospy.Subscriber(
            args.odom_topic, Odometry, self.odom_callback, queue_size=10
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / args.rc_rate), self.timer_callback
        )

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        with self.lock:
            self.current_pos = (pos.x, pos.y, pos.z)
            self.current_vel = (vel.x, vel.y, vel.z)
            self.last_odom_time = rospy.Time.now()

    def start_rc_pulse(self):
        with self.lock:
            if self.rc_pulse_deadline is not None:
                print("RC pulse is already active; please wait.")
                return
            self.rc_msg.channels[9] = self.RC_PULSE_LOW
            self.rc_pulse_deadline = time.monotonic() + self.RC_PULSE_SECONDS
        print(
            "RC channel 10 pulse started: {} for {:.1f}s, then {}.".format(
                self.RC_PULSE_LOW, self.RC_PULSE_SECONDS, self.RC_PULSE_HIGH
            )
        )

    def set_pd_target(self, x, y, z, yaw):
        with self.lock:
            self.current_target = (x, y, z, yaw)
        print(
            "PD target set: x={:g}, y={:g}, z={:g}, yaw={:g}; "
            "max velocity={:g} m/s.".format(
                x, y, z, yaw, self.args.max_vel
            )
        )

    def stop_pd(self):
        with self.lock:
            was_active = self.current_target is not None
            self.current_target = None
        print("PD target cleared." if was_active else "No active PD target.")

    def publish_goal(self, x, y, z, yaw):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.args.frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(goal)
        print(
            "Goal published: x={:g}, y={:g}, z={:g}, yaw={:g} "
            "(frame={}, topic={}).".format(
                x, y, z, yaw, self.args.frame, self.args.goal_topic
            )
        )

    def odom_is_fresh(self, now, last_odom_time):
        if last_odom_time is None:
            return False
        return (now - last_odom_time).to_sec() <= self.args.odom_timeout

    def build_pd_command(self, target, position, velocity, stamp):
        tx, ty, tz, yaw = target
        ex = tx - position[0]
        ey = ty - position[1]
        ez = tz - position[2]

        vx = self.args.kp * ex - self.args.kd * velocity[0]
        vy = self.args.kp * ey - self.args.kd * velocity[1]
        vz = self.args.kp * ez - self.args.kd * velocity[2]

        velocity_norm = math.sqrt(vx * vx + vy * vy + vz * vz)
        if velocity_norm > self.args.max_vel:
            scale = self.args.max_vel / velocity_norm
            vx *= scale
            vy *= scale
            vz *= scale

        cmd = PositionCommand()
        cmd.header.stamp = stamp
        cmd.header.frame_id = self.args.frame
        cmd.position.x = tx
        cmd.position.y = ty
        cmd.position.z = tz
        cmd.velocity.x = vx
        cmd.velocity.y = vy
        cmd.velocity.z = vz
        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = 0.0
        cmd.yaw = yaw
        return cmd

    def timer_callback(self, _event):
        now = rospy.Time.now()
        with self.lock:
            if (
                self.rc_pulse_deadline is not None
                and time.monotonic() >= self.rc_pulse_deadline
            ):
                self.rc_msg.channels[9] = self.RC_PULSE_HIGH
                self.rc_pulse_deadline = None

            self.rc_msg.header.stamp = now
            rc_msg = self.rc_msg
            target = self.current_target
            position = self.current_pos
            velocity = self.current_vel
            last_odom_time = self.last_odom_time

        self.rc_pub.publish(rc_msg)

        if target is None:
            return
        if not self.odom_is_fresh(now, last_odom_time):
            rospy.logwarn_throttle(
                2.0,
                "PD target is active, but odometry on %s is missing or stale.",
                self.args.odom_topic,
            )
            return

        dx = position[0] - target[0]
        dy = position[1] - target[1]
        dz = position[2] - target[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance <= self.args.reach_distance:
            with self.lock:
                if self.current_target == target:
                    self.current_target = None
            print(
                "\nReached target within {:g} m; PD command stopped.".format(
                    self.args.reach_distance
                )
            )
            return

        self.cmd_pub.publish(
            self.build_pd_command(target, position, velocity, now)
        )

    def print_status(self):
        with self.lock:
            position = self.current_pos
            velocity = self.current_vel
            target = self.current_target
            last_odom_time = self.last_odom_time
            pulse_active = self.rc_pulse_deadline is not None

        print("RC topic:       {} @ {:g} Hz".format(
            self.args.rc_topic, self.args.rc_rate
        ))
        print("Odometry topic: {}".format(self.args.odom_topic))
        print("Command topic:  {}".format(self.args.cmd_topic))
        print("Goal topic:     {}".format(self.args.goal_topic))
        print("Frame:          {}".format(self.args.frame))
        print("RC pulse:       {}".format("active" if pulse_active else "idle"))
        print("Position:       {}".format(position if position is not None else "no data"))
        print("Velocity:       {}".format(velocity if velocity is not None else "no data"))
        print("PD target:      {}".format(target if target is not None else "none"))
        if last_odom_time is not None:
            age = max(0.0, (rospy.Time.now() - last_odom_time).to_sec())
            print("Odometry age:   {:.3f} s".format(age))

    @staticmethod
    def print_help():
        print(
            "\nCommands:\n"
            "  1 | rc                  trigger the 2-second RC channel-10 pulse\n"
            "  2 | home                PD target (0, 0, 1.5, yaw=0)\n"
            "  3 | task                PD target (0.5, 4, 1, yaw=0)\n"
            "  pd X Y Z [YAW]          set an arbitrary PD flight target\n"
            "  gl X Y Z [YAW]          publish a PoseStamped goal\n"
            "  stop                    stop publishing PD commands\n"
            "  status                  show topics, odometry, and active state\n"
            "  help | ?                show this help\n"
            "  quit | exit | q | e     exit\n"
            "\nYAW is in radians and defaults to 0.\n"
            "The gl command does not automatically enable PD control."
        )

    @staticmethod
    def parse_pose_values(parts, command):
        if len(parts) not in (4, 5):
            raise ValueError("Usage: {} X Y Z [YAW]".format(command))
        try:
            values = [float(value) for value in parts[1:]]
        except ValueError:
            raise ValueError("X, Y, Z and YAW must be numbers")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("X, Y, Z and YAW must be finite numbers")
        if len(values) == 3:
            values.append(0.0)
        return values

    def run(self):
        print("\nUnified UAV CLI")
        print("Make sure px4ctrl is in sim mode before using RC/PD control.")
        self.print_help()

        while not rospy.is_shutdown():
            try:
                line = input("\n>>> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not line:
                continue
            try:
                parts = shlex.split(line)
                command = parts[0].lower()

                if command in ("quit", "exit", "q", "e"):
                    break
                if command in ("help", "?"):
                    self.print_help()
                elif command in ("1", "rc"):
                    self.start_rc_pulse()
                elif command in ("2", "home"):
                    self.set_pd_target(*self.HOME_TARGET)
                elif command in ("3", "task"):
                    self.set_pd_target(*self.TASK_TARGET)
                elif command == "pd":
                    self.set_pd_target(*self.parse_pose_values(parts, "pd"))
                elif command == "gl":
                    self.publish_goal(*self.parse_pose_values(parts, "gl"))
                elif command == "stop":
                    self.stop_pd()
                elif command == "status":
                    self.print_status()
                else:
                    print("Unknown command: {}. Type 'help'.".format(command))
            except ValueError as exc:
                print(exc)

        rospy.signal_shutdown("CLI exited")
        print("Unified UAV CLI stopped.")


def main():
    try:
        args = parse_args()
        validate_args(args)
    except ValueError as exc:
        print("Invalid option: {}".format(exc), file=sys.stderr)
        return 2

    try:
        rospy.init_node("uav_cli")
        UavCli(args).run()
        return 0
    except rospy.ROSInterruptException:
        return 0
    except Exception as exc:
        print("UAV CLI failed: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
