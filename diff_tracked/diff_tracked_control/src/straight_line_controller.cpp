#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>

namespace diff_tracked_control{

class RegulatedPurePursuitController{
    public:
        RegulatedPurePursuitController() : private_nh_("~") {}

        bool Initialize(){
            LoadParameters();

            if(!ValidateParameters()){
                return false;
            }
            //公共订阅，绝对话题
            cmd_vel_publisher_ = 
            nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

            planned_path_publisher_ = 
            nh_.advertise<nav_msgs::Path>(planned_path_topic_, 1, true);

            actual_path_publisher_ = 
            nh_.advertise<nav_msgs::Path>(actual_path_topic_, 1, true);

            odom_subscriber_ = nh_.subscribe(
                odom_topic_, 1,
                &RegulatedPurePursuitController::OdomCallback, this);
            
            control_timer_ = nh_.createTimer(
                ros::Duration(1.0 / control_rate_),
                &RegulatedPurePursuitController::ControlCallback, this);

            return true;
        }

    private:
        
        void LoadParameters(){
            //私有参数
            private_nh_.param("odom_topic", odom_topic_, std::string("/odom"));
            private_nh_.param("cmd_vel_topic",cmd_vel_topic_,
                             std::string("/cmd_vel"));
            private_nh_.param("planned_path_topic", planned_path_topic_,
                             std::string("/planned_path"));
            private_nh_.param("actual_path_topic", actual_path_topic_,
                             std::string("/actual_path"));

            private_nh_.param("target_distance", target_distance_, 5.0);
            private_nh_.param("cruise_speed", desired_speed_, 0.5);
            private_nh_.param("min_linear_speed", min_linear_speed_, 0.05);
            private_nh_.param("max_angular_speed", max_angular_speed_, 1.0);
            private_nh_.param("distance_tolerance", distance_tolerance_, 0.03);
            private_nh_.param("control_rate", control_rate_, 20.0);

            private_nh_.param("lookahead_distance", lookahead_distance_, 0.5);
            private_nh_.param("min_lookahead_distance",
                            min_lookahead_distance_, 0.25);
            private_nh_.param("max_lookahead_distance",
                            max_lookahead_distance_, 1.0);
            private_nh_.param("lookahead_time", lookahead_time_, 1.0);

            private_nh_.param("regulated_min_radius",
                            regulated_min_radius_, 0.9);
            private_nh_.param("max_lateral_accel", max_lateral_accel_, 0.8);
            private_nh_.param("approach_distance", approach_distance_, 0.8);

            private_nh_.param("max_linear_accel", max_linear_accel_, 0.6);
            private_nh_.param("max_linear_decel", max_linear_decel_, 0.8);
            private_nh_.param("max_angular_accel", max_angular_accel_, 2.0);

            private_nh_.param("odom_timeout", odom_timeout_, 0.5);

            private_nh_.param("planned_path_resolution",
                            planned_path_resolution_, 0.25);
            private_nh_.param("actual_path_min_distance",
                            actual_path_min_distance_, 0.05);
            private_nh_.param("actual_path_max_poses",
                            actual_path_max_poses_, 5000);

        }

        bool ValidateParameters() const {
            if(!CheckPositive("target_distance", target_distance_)) {
                return false;
            }

            if (!CheckPositive("cruise_speed", desired_speed_)) {
                return false;
            }
            
            if (!CheckPositive("max_angular_speed", max_angular_speed_)) {
                return false;
            }

            if (!CheckPositive("distance_tolerance", distance_tolerance_)) {
                return false;
            }

            if (!CheckPositive("control_rate", control_rate_)) {
                return false;
            }

            if (!CheckPositive("lookahead_distance", lookahead_distance_)) {
                return false;
            }

            if (!CheckPositive("min_lookahead_distance",
                            min_lookahead_distance_)) {
                return false;
            }

            if (!CheckPositive("max_lookahead_distance",
                            max_lookahead_distance_)) {
                return false;
            }

            if (!CheckPositive("regulated_min_radius",
                            regulated_min_radius_)) {
                return false;
            }

            if (!CheckPositive("max_lateral_accel", max_lateral_accel_)) {
                return false;
            }

            if (!CheckPositive("approach_distance", approach_distance_)) {
                return false;
            }

            if (!CheckPositive("max_linear_accel", max_linear_accel_)) {
                return false;
            }

            if (!CheckPositive("max_linear_decel", max_linear_decel_)) {
                return false;
            }

            if (!CheckPositive("max_angular_accel", max_angular_accel_)) {
                return false;
            }

            if (!CheckPositive("odom_timeout", odom_timeout_)) {
                return false;
            }

            if (!CheckPositive("planned_path_resolution",
                            planned_path_resolution_)) {
                return false;
            }

            if (!CheckPositive("actual_path_min_distance",
                            actual_path_min_distance_)) {
                return false;
            }

            if (min_linear_speed_ < 0.0 ||
                min_linear_speed_ > desired_speed_) {
              ROS_ERROR("~min_linear_speed must be between zero and "
                        "~cruise_speed");
                return false;
            }

                if (min_lookahead_distance_ > max_lookahead_distance_) {
              ROS_ERROR("~min_lookahead_distance must not exceed "
                        "~max_lookahead_distance");
                return false;
            }

            if (lookahead_time_ < 0.0) {
              ROS_ERROR("~lookahead_time must be non-negative");
                return false;
            }

            if (actual_path_max_poses_ < 2) {
              ROS_ERROR("~actual_path_max_poses must be at least 2");
                return false;
            }
             return true;
        }
        bool CheckPositive(const std::string& name, double value) const {
            if (value > 0.0) {
                return true;
            }   
            ROS_ERROR_STREAM("~" << name << " must be positive");
                return false;
        }

        double Clamp(double value, double lower, double upper) const {
            return std::max(lower, std::min(upper, value));
        }

        double Approach(double value, double target, double max_delta) const {
            return value + Clamp(target - value, -max_delta, max_delta);
        }

        double YawFromQuaternion(
            const geometry_msgs::Quaternion& quaternion) const {
            const double siny_cosp =
                2.0 * (quaternion.w * quaternion.z +
                    quaternion.x * quaternion.y);

            const double cosy_cosp =
                1.0 - 2.0 * (quaternion.y * quaternion.y +
                            quaternion.z * quaternion.z);

            return std::atan2(siny_cosp, cosy_cosp);
        }
        
        ros::Time MessageStamp(const std_msgs::Header& header) const {
            if (header.stamp.isZero()) {
                return ros::Time::now();
            }
            return header.stamp;
        }

        void OdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
            latest_odom_ = *message;
            has_latest_odom_ = true;

            if(!has_start_pose_){
                start_x_ = message->pose.pose.position.x;
                start_y_ = message->pose.pose.position.y;
                start_yaw_ = 
                YawFromQuaternion(message->pose.pose.orientation);

                has_start_pose_ = true;

                ROS_INFO("RPP start at (%.3f, %.3f), yaw %.3f rad",
                start_x_, start_y_, start_yaw_);

                PublishPlannedPath(*message);
            }

            if (!finished_) {
                RecordActualPath(*message);
            }
        }

        void PublishPlannedPath(const nav_msgs::Odometry& odom) {
            //如果里程计消息的 frame_id 不为空，就使用它,否则默认odom
            const std::string frame_id = 
                odom.header.frame_id.empty() ? "odom" : odom.header.frame_id;
            
            const ros::Time stamp = MessageStamp(odom.header);

            const int segment_count = std::max(
                1, static_cast<int>(
                    std::ceil(target_distance_ / planned_path_resolution_))); 
            
            nav_msgs::Path path;
            path.header.frame_id = frame_id;
            path.header.stamp = stamp;

            for(int index = 0; index <= segment_count; ++index){
                const double distance =
                    target_distance_ * static_cast<double>(index) /
                    static_cast<double>(segment_count);
                
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = frame_id;
                pose.header.stamp = stamp;

                pose.pose.position.x = 
                    start_x_ + distance * std::cos(start_yaw_);
                pose.pose.position.y = 
                    start_y_ + distance * std::sin(start_yaw_);
                
                pose.pose.orientation.z = std::sin(start_yaw_ * 0.5);
                pose.pose.orientation.w = std::cos(start_yaw_ * 0.5);

                path.poses.push_back(pose);
            }

            planned_path_publisher_.publish(path);

            ROS_INFO("Published %.1f m planned path with %zu poses",
                target_distance_, path.poses.size());
        }

        void RecordActualPath(const nav_msgs::Odometry& odom){
            const geometry_msgs::Point& position = odom.pose.pose.position;

            if(has_last_actual_position_){
                const double dx = position.x - last_actual_x_;
                const double dy = position.y - last_actual_y_;

                if(std::hypot(dx, dy) < actual_path_min_distance_){
                    return;
                }
            }

            const std::string frame_id = 
                odom.header.frame_id.empty() ? "odom" : odom.header.frame_id;

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = frame_id;
            pose.header.stamp = MessageStamp(odom.header);
            pose.pose = odom.pose.pose;

            actual_path_.header.frame_id = frame_id;
            actual_path_.header.stamp = pose.header.stamp;
            actual_path_.poses.push_back(pose);

            if(actual_path_.poses.size() >
                static_cast<std::size_t>(actual_path_max_poses_)){
                actual_path_.poses.erase(actual_path_.poses.begin());
            }
            
            last_actual_x_ = position.x;
            last_actual_y_ = position.y;
            has_last_actual_position_ = true;

            actual_path_publisher_.publish(actual_path_);
        }
     
        void ControlCallback(const ros::TimerEvent& event) {
            if(!has_latest_odom_ || !has_start_pose_) {
                ROS_WARN_THROTTLE(2.0, "waiting for odometry on %s",
                                  odom_topic_.c_str());
                return;
            }
            //最近里程计消息的时间
            const ros::Time odom_stamp = latest_odom_.header.stamp;

            if(!odom_stamp.isZero()) {
                //信息年龄
                const double odom_age = 
                    (ros::Time::now() - odom_stamp).toSec();
                
                if(odom_age > odom_timeout_) {
                    //相同警告最多每 2 秒输出一次
                    ROS_WARN_THROTTLE(2.0,
                          "Odometry is stale; commanding stop");
                    PublishStop();
                    return;
                }
            }

            if(finished_) {
                PublishStop();
                return;
            }

            double control_period = 1.0 / control_rate_;

            if(!event.last_real.isZero()) {
                //
                control_period =
                    (event.current_real - event.last_real).toSec();
            }

            if(control_period <= 0.0) {
                return;
            }

            const geometry_msgs::Pose& robot_pose = 
                latest_odom_.pose.pose;

            const double robot_x = robot_pose.position.x;
            const double robot_y = robot_pose.position.y;
            const double robot_yaw = 
                YawFromQuaternion(latest_odom_.pose.pose.orientation);
            
            const double dx = robot_x - start_x_;
            const double dy = robot_y - start_y_;

            const double along_track = 
                std::cos(start_yaw_) * dx +
                std::sin(start_yaw_) * dy;

            const double cross_track =
                -std::sin(start_yaw_) * dx +
                std::cos(start_yaw_) * dy;

            const double remaining = 
                target_distance_ - along_track;

            const double target_x =
            start_x_ + target_distance_ * std::cos(start_yaw_);

            const double target_y =
                start_y_ + target_distance_ * std::sin(start_yaw_);

            const double goal_dx = target_x - robot_x;
            const double goal_dy = target_y - robot_y;
            const double goal_distance = std::hypot(goal_dx, goal_dy);

            if(goal_distance  <= distance_tolerance_ &&
                std::abs(cross_track) <= 0.20) {
                finished_ = true;
                PublishStop();
  
                ROS_INFO("Target reached: goal distance=%.3f m, "
                         "along=%.3f m, lateral=%.3f m",
                          goal_distance, along_track, cross_track);
                return;
            }

            const double measured_speed = 
                std::abs(latest_odom_.twist.twist.linear.x);
            
            double lookahead =
                lookahead_distance_ + lookahead_time_ * measured_speed;

            lookahead = Clamp(lookahead,
                              min_lookahead_distance_,
                              max_lookahead_distance_);
            //计算前视点在线路上的距离
            const double carrot_distance = 
                Clamp(along_track + lookahead, 
                      0.0, 
                      target_distance_);
            //前视点二维坐标
            const double carrot_x = 
                start_x_ + carrot_distance * std::cos(start_yaw_);
            const double carrot_y =
                start_y_ + carrot_distance * std::sin(start_yaw_);  

            const double carrot_dx = carrot_x - robot_x;
            const double carrot_dy = carrot_y - robot_y;
            //前视点距离(机器人坐标系)
            const double carrot_robot_x = 
                std::cos(robot_yaw) * carrot_dx +
                std::sin(robot_yaw) * carrot_dy;

            const double carrot_robot_y =
                -std::sin(robot_yaw) * carrot_dx +
                std::cos(robot_yaw) * carrot_dy;

            const double carrot_robot_distance_squared = 
                carrot_robot_x * carrot_robot_x + 
                carrot_robot_y * carrot_robot_y;

            double curvature = 0.0;
            const double curvature_epsilon = 1.0e-8;

            if(carrot_robot_distance_squared > curvature_epsilon) {
                curvature = 
                2.0 * carrot_robot_y / carrot_robot_distance_squared;
            }

            double target_linear_speed = desired_speed_;
            const double absolute_curvature = std::abs(curvature);

            if(absolute_curvature > curvature_epsilon) {
                const double turning_radius =
                1.0 / absolute_curvature;

                if(turning_radius < regulated_min_radius_) {
                    target_linear_speed = 
                    turning_radius / regulated_min_radius_;
                }

                const double lateral_acceleration_speed =
                    std::sqrt(max_lateral_accel_ / absolute_curvature);
                //两个限制
                target_linear_speed =
                    std::min(target_linear_speed, 
                        lateral_acceleration_speed);
            }

            if(remaining < approach_distance_) {
                target_linear_speed *= remaining / approach_distance_;
            }

            target_linear_speed = 
                Clamp(target_linear_speed, 
                      min_linear_speed_, 
                      desired_speed_);
            
            if(absolute_curvature > curvature_epsilon) {
                target_linear_speed = 
                    std::min(target_linear_speed, 
                             max_angular_speed_ / absolute_curvature);
            }
            //
            const double linear_acceleration = 
                target_linear_speed >= last_linear_command_ 
                    ? max_linear_accel_
                    : max_linear_decel_;
            //最多变化指定步长
            const double linear_command =
                Approach(last_linear_command_, 
                         target_linear_speed, 
                         linear_acceleration * control_period);

            const double target_angular_command =
                Clamp(linear_command * curvature,
                      -max_angular_speed_,
                       max_angular_speed_);
            
            const double angular_command =
                Approach(last_angular_command_,
                         target_angular_command,
                         max_angular_accel_ * control_period);

            geometry_msgs::Twist command;
            command.linear.x = linear_command;
            command.angular.z = angular_command;

            cmd_vel_publisher_.publish(command);

            last_linear_command_ = linear_command;
            last_angular_command_ = angular_command;
            
            ROS_INFO_THROTTLE(
            1.0,
            "RPP along %.2f/%.2f m, lateral %.3f m, "
            "lookahead %.2f m, curvature %.3f 1/m, "
            "cmd (%.2f m/s, %.2f rad/s)",
            along_track,
            target_distance_,
            cross_track,
            lookahead,
            curvature,
            linear_command,
            angular_command);
        }

        void PublishStop() {
            last_linear_command_ = 0.0;
            last_angular_command_ = 0.0;

            geometry_msgs::Twist stop_command;
            cmd_vel_publisher_.publish(stop_command);
        }

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Publisher cmd_vel_publisher_;
        ros::Publisher planned_path_publisher_;
        ros::Publisher actual_path_publisher_;

        ros::Subscriber odom_subscriber_;
        ros::Timer control_timer_;

        std::string odom_topic_;
        std::string cmd_vel_topic_;
        std::string planned_path_topic_;
        std::string actual_path_topic_;

        double target_distance_ = 5.0;
        double desired_speed_ = 0.5;
        double min_linear_speed_ = 0.05;
        double max_angular_speed_ = 1.0;
        double distance_tolerance_ = 0.03;
        double control_rate_ = 20.0;

        double lookahead_distance_ = 0.5;
        double min_lookahead_distance_ = 0.25;
        double max_lookahead_distance_ = 1.0;
        double lookahead_time_ = 1.0;

        double regulated_min_radius_ = 0.9;
        double max_lateral_accel_ = 0.8;
        double approach_distance_ = 0.8;

        double max_linear_accel_ = 0.6;
        double max_linear_decel_ = 0.8;
        double max_angular_accel_ = 2.0;

        double odom_timeout_ = 0.5;

        double planned_path_resolution_ = 0.25;
        double actual_path_min_distance_ = 0.05;
        int actual_path_max_poses_ = 5000;

        bool has_start_pose_ = false;
        bool has_latest_odom_ = false;
        bool finished_ = false;

        double start_x_ = 0.0;
        double start_y_ = 0.0;
        double start_yaw_ = 0.0;

        nav_msgs::Odometry latest_odom_;
        nav_msgs::Path actual_path_;

        double last_linear_command_ = 0.0;
        double last_angular_command_ = 0.0;

        bool has_last_actual_position_ = false;
        double last_actual_x_ = 0.0;
        double last_actual_y_ = 0.0;
};
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "regulated_pure_pursuit_controller");
    diff_tracked_control::RegulatedPurePursuitController controller;
    if(!controller.Initialize())
    {
        ROS_FATAL("Failed to initialize RPP controller");
        return 1;
    }

    ros::spin();
    return 0;
}

