#!/usr/bin/env python3
import rospy
import threading
import time
import math
from mavros_msgs.msg import RCIn
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry

# PID parameters.
KP = 1.0
KD = 0.5
REACH_DISTANCE = 0.5
MAX_VEL = 5.0

# Target point for input '3'.
TASK_TARGET_X = 0.5
TASK_TARGET_Y = 4.0
TASK_TARGET_Z = 1.0
TASK_TARGET_YAW = 0.0

PURPLE = "\033[35m"
RESET = "\033[0m"

InitPose = PositionCommand()
InitPose.position.x = 0.0
InitPose.position.y = 0.0
InitPose.position.z = 1.5
InitPose.velocity.x = 0.0
InitPose.velocity.y = 0.0
InitPose.velocity.z = 0.0
InitPose.acceleration.x = 0.0
InitPose.acceleration.y = 0.0
InitPose.acceleration.z = 0.0
InitPose.yaw = 0.0


TaskPose = PositionCommand()
TaskPose.position.x = TASK_TARGET_X
TaskPose.position.y = TASK_TARGET_Y
TaskPose.position.z = TASK_TARGET_Z
TaskPose.velocity.x = 0.0
TaskPose.velocity.y = 0.0
TaskPose.velocity.z = 0.0
TaskPose.acceleration.x = 0.0
TaskPose.acceleration.y = 0.0
TaskPose.acceleration.z = 0.0
TaskPose.yaw = TASK_TARGET_YAW


class RCTrigger:
    def __init__(self):
        self.rc_msg = RCIn()
        self.rc_msg.channels = [1495,1495,1495,1495,1995,1995,1495,1495,1495,1945]   # 通常RC有8个通道，索引0-7对应通道1-8
        
        self.takeoff_flag = False
        self.takeoff_start_time = 0
        self.current_target = None
        self.current_pos = None
        self.current_vel = None

        self.pub = rospy.Publisher('/mavros/rc/in_sim', RCIn, queue_size=10)
        self.cmdpub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=10)
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback, queue_size=10)
        
        self.timer = rospy.Timer(rospy.Duration(1/30), self.timer_callback)
        
        print("\033[4;32m注意将px4ctrl设置为sim模式,接收rc_sim.\033[0m")
        print(
            "\033[4;32m输入'1':起飞或降落\n"
            f"输入'2':PD发送(0,0,1.5)直到距离目标{REACH_DISTANCE:.1f}m内\n"
            f"输入'3':PD发送({TASK_TARGET_X:g},{TASK_TARGET_Y:g},{TASK_TARGET_Z:g})直到距离目标{REACH_DISTANCE:.1f}m内\n"
            "输入'e''q''Ctrl+C'：退出程序.\033[0m"
        )

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        self.current_pos = (pos.x, pos.y, pos.z)
        self.current_vel = (vel.x, vel.y, vel.z)

    def reached_target(self):
        if self.current_target is None or self.current_pos is None:
            return False

        dx = self.current_pos[0] - self.current_target.position.x
        dy = self.current_pos[1] - self.current_target.position.y
        dz = self.current_pos[2] - self.current_target.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz) <= REACH_DISTANCE

    def build_pd_command(self):
        if self.current_target is None or self.current_pos is None or self.current_vel is None:
            return None

        ex = self.current_target.position.x - self.current_pos[0]
        ey = self.current_target.position.y - self.current_pos[1]
        ez = self.current_target.position.z - self.current_pos[2]

        vx = KP * ex - KD * self.current_vel[0]
        vy = KP * ey - KD * self.current_vel[1]
        vz = KP * ez - KD * self.current_vel[2]

        vel_norm = math.sqrt(vx * vx + vy * vy + vz * vz)
        if vel_norm > MAX_VEL and vel_norm > 1e-6:
            scale = MAX_VEL / vel_norm
            vx *= scale
            vy *= scale
            vz *= scale

        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.position.x = self.current_target.position.x
        cmd.position.y = self.current_target.position.y
        cmd.position.z = self.current_target.position.z
        cmd.velocity.x = vx
        cmd.velocity.y = vy
        cmd.velocity.z = vz
        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = 0.0
        cmd.yaw = self.current_target.yaw
        return cmd

    def timer_callback(self, event):
        # 检查是否需要临时修改通道10
        if self.takeoff_flag:
            current_time = time.time()
            if current_time - self.takeoff_start_time < 2.0:
                self.rc_msg.channels[9] =   1495
            else:
                self.rc_msg.channels[9] =   1995
                self.takeoff_flag = False

        self.pub.publish(self.rc_msg)

        if self.current_target is None:
            return

        if self.reached_target():
            print(f"{PURPLE}目标点{REACH_DISTANCE:.1f}m内，停止发送 position_cmd{RESET}")
            self.current_target = None
            return

        cmd = self.build_pd_command()
        if cmd is not None:
            self.cmdpub.publish(cmd)

    def handle_user_input(self):
        while not rospy.is_shutdown():
            user_input = input(">>>input:").strip().lower()
            if user_input == "1":
                if not self.takeoff_flag:
                    self.takeoff_flag = True
                    self.takeoff_start_time = time.time()
                    print("get takeoff!")
                else:
                    print("wait takeoff!")
            elif user_input == "2":
                self.current_target = InitPose
                print(f"PD发送 (0,0,1.5)，最大速度 {MAX_VEL:.1f} m/s")

            elif user_input == "3":
                self.current_target = TaskPose
                print(f"PD发送 ({TASK_TARGET_X:g},{TASK_TARGET_Y:g},{TASK_TARGET_Z:g})，最大速度 {MAX_VEL:.1f} m/s")

            elif user_input in ["e", "q"]:
                print("程序退出")
                rospy.signal_shutdown("exit~")
                break
            else:
                pass

if __name__ == '__main__':
    try:
        rospy.init_node('pub_rc', anonymous=True)
        
        rc_controller = RCTrigger()
        
        input_thread = threading.Thread(target=rc_controller.handle_user_input)
        input_thread.daemon = True
        input_thread.start()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print("程序被ROS中断")
    except Exception as e:
        print(f"发生错误: {str(e)}")
