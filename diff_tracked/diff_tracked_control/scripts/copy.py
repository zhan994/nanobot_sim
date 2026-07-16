import rospy
import math

class StraightLineController:
    "main controller for straight line"
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic","/odom")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic","/cmd_vel")
        self.target_distance = rospy.get_param("~target_distance",5.0)
        self.cruise_speed = rospy.get_param("~cruise_speed",0.5)
        self.max_angular_speed = rospy.get_param("~max_angular_speed",1.0)
        self.distance_tolerance = rospy.get_param("~distance_tolerance",0.03)   
        self.control_rate = rospy.get_param("~control_rate",20.0)
        self.lookahead_distance = rospy.get_param("~lookahead_distance",0.5)

        if self.target_distance <= 0.0:
            raise rospy.ROSInitException("~target_distance must be positive")
        if self.cruise_speed <= 0.0:
            raise rospy.ROSInitException("~cruise_speed must be positive")
        if self.control_rate <= 0.0:
            raise rospy.ROSInitException("~control_rate must be positive")
        if self.lookahead_distance <= 0.0:
            raise rospy.ROSInitException("~lookahead_distance must be positive")
        
        


if __name__ == "__main__":
    rospy.init_node("Straight_Line_Controller")
    try:
        StraightLineController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
