#include <cmath>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace diff_tracked_base {

class VirtualEncoderOdometry {
 public:
  VirtualEncoderOdometry() : private_nh_("~") {}

  bool Initialize() {
    private_nh_.param("input_odom_topic", input_odom_topic_,
                      std::string("/gz_odom"));
    private_nh_.param("output_odom_topic", output_odom_topic_,
                      std::string("/encoder_odom"));
    private_nh_.param("joint_states_topic", joint_states_topic_,
                      std::string("/virtual_encoder/joint_states"));
    private_nh_.param("odom_frame", odom_frame_, std::string("odom"));
    private_nh_.param("base_frame", base_frame_, std::string("base_link"));
    private_nh_.param("left_joint_name", left_joint_name_,
                      std::string("virtual_left_encoder_joint"));
    private_nh_.param("right_joint_name", right_joint_name_,
                      std::string("virtual_right_encoder_joint"));

    private_nh_.param("wheel_separation", wheel_separation_, 0.699);
    private_nh_.param("wheel_radius", wheel_radius_, 0.051);
    private_nh_.param("max_dt", max_dt_, 0.2);
    private_nh_.param("lateral_velocity_tolerance",
                      lateral_velocity_tolerance_, 2.0e-2);

    private_nh_.param("pose_covariance_xy", pose_covariance_xy_, 1.0e-3);
    private_nh_.param("pose_covariance_yaw", pose_covariance_yaw_, 1.0e-3);
    private_nh_.param("twist_covariance_x", twist_covariance_x_, 1.0e-3);
    private_nh_.param("twist_covariance_y", twist_covariance_y_, 1.0e-3);
    private_nh_.param("twist_covariance_yaw", twist_covariance_yaw_, 1.0e-3);
    private_nh_.param("unobserved_variance", unobserved_variance_, 1.0e6);

    if (wheel_separation_ <= 0.0 || wheel_radius_ <= 0.0 || max_dt_ <= 0.0) {
      ROS_ERROR("wheel_separation, wheel_radius and max_dt must be positive");
      return false;
    }

    odom_publisher_ =
        nh_.advertise<nav_msgs::Odometry>(output_odom_topic_, 10);
    joint_states_publisher_ =
        nh_.advertise<sensor_msgs::JointState>(joint_states_topic_, 10);
    odom_subscriber_ =
        nh_.subscribe(input_odom_topic_, 20,
                      &VirtualEncoderOdometry::OdomCallback, this);

    ROS_INFO("Virtual encoder: %s -> %s, wheel separation %.4f m, radius "
             "%.4f m",
             input_odom_topic_.c_str(), output_odom_topic_.c_str(),
             wheel_separation_, wheel_radius_);
    return true;
  }

 private:
  static double NormalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void Reset(const ros::Time& stamp) {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    left_wheel_position_ = 0.0;
    right_wheel_position_ = 0.0;
    last_stamp_ = stamp;
    initialized_ = true;
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
      const ros::Time stamp =
          message->header.stamp.isZero() ? ros::Time::now()
                                        : message->header.stamp;
      const double linear_velocity = message->twist.twist.linear.x;
      const double angular_velocity = message->twist.twist.angular.z;

      if (!std::isfinite(linear_velocity) ||
          !std::isfinite(angular_velocity)) {
        ROS_WARN_THROTTLE(1.0, "Virtual encoder ignored non-finite velocity");
        return;
      }

      if (std::abs(message->twist.twist.linear.y) >
          lateral_velocity_tolerance_) {
        ROS_WARN_THROTTLE(
            5.0,
            "Virtual differential encoder ignores lateral velocity %.4f m/s",
            message->twist.twist.linear.y);
      }

      const double left_wheel_velocity =
          (linear_velocity - angular_velocity * wheel_separation_ * 0.5) /
          wheel_radius_;
      const double right_wheel_velocity =
          (linear_velocity + angular_velocity * wheel_separation_ * 0.5) /
          wheel_radius_;

      if (!initialized_) {
        Reset(stamp);
        Publish(stamp, left_wheel_velocity, right_wheel_velocity);
        return;
      }

      const double dt = (stamp - last_stamp_).toSec();
      if (dt < 0.0) {
        ROS_WARN("Simulation time moved backwards; resetting virtual encoder");
        Reset(stamp);
        Publish(stamp, left_wheel_velocity, right_wheel_velocity);
        return;
      }

      if (dt > max_dt_) {
        ROS_WARN_THROTTLE(1.0,
                          "Virtual encoder skipped %.3f s integration gap",
                          dt);
        last_stamp_ = stamp;
        Publish(stamp, left_wheel_velocity, right_wheel_velocity);
        return;
      }

      if (dt > 0.0) {
        const double left_delta_angle = left_wheel_velocity * dt;
        const double right_delta_angle = right_wheel_velocity * dt;
        left_wheel_position_ += left_delta_angle;
        right_wheel_position_ += right_delta_angle;

        const double left_distance = wheel_radius_ * left_delta_angle;
        const double right_distance = wheel_radius_ * right_delta_angle;
        const double center_distance = 0.5 * (left_distance + right_distance);
        const double heading_delta =
            (right_distance - left_distance) / wheel_separation_;
        const double midpoint_heading = yaw_ + 0.5 * heading_delta;

        x_ += center_distance * std::cos(midpoint_heading);
        y_ += center_distance * std::sin(midpoint_heading);
        yaw_ = NormalizeAngle(yaw_ + heading_delta);
        last_stamp_ = stamp;
      }

      Publish(stamp, left_wheel_velocity, right_wheel_velocity);
  }

  void Publish(const ros::Time& stamp, double left_wheel_velocity,
               double right_wheel_velocity) {
    const double encoder_linear_velocity =
        0.5 * wheel_radius_ *
        (left_wheel_velocity + right_wheel_velocity);
    const double encoder_angular_velocity =
        wheel_radius_ * (right_wheel_velocity - left_wheel_velocity) /
        wheel_separation_;

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(0.5 * yaw_);
    odom.pose.pose.orientation.w = std::cos(0.5 * yaw_);
    odom.twist.twist.linear.x = encoder_linear_velocity;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = encoder_angular_velocity;

    odom.pose.covariance[0] = pose_covariance_xy_;
    odom.pose.covariance[7] = pose_covariance_xy_;
    odom.pose.covariance[14] = unobserved_variance_;
    odom.pose.covariance[21] = unobserved_variance_;
    odom.pose.covariance[28] = unobserved_variance_;
    odom.pose.covariance[35] = pose_covariance_yaw_;

    odom.twist.covariance[0] = twist_covariance_x_;
    odom.twist.covariance[7] = twist_covariance_y_;
    odom.twist.covariance[14] = unobserved_variance_;
    odom.twist.covariance[21] = unobserved_variance_;
    odom.twist.covariance[28] = unobserved_variance_;
    odom.twist.covariance[35] = twist_covariance_yaw_;
    odom_publisher_.publish(odom);

    sensor_msgs::JointState joint_states;
    joint_states.header.stamp = stamp;
    joint_states.header.frame_id = base_frame_;
    joint_states.name.push_back(left_joint_name_);
    joint_states.name.push_back(right_joint_name_);
    joint_states.position.push_back(left_wheel_position_);
    joint_states.position.push_back(right_wheel_position_);
    joint_states.velocity.push_back(left_wheel_velocity);
    joint_states.velocity.push_back(right_wheel_velocity);
    joint_states_publisher_.publish(joint_states);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher odom_publisher_;
  ros::Publisher joint_states_publisher_;
  ros::Subscriber odom_subscriber_;

  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string joint_states_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string left_joint_name_;
  std::string right_joint_name_;

  double wheel_separation_{0.699};
  double wheel_radius_{0.051};
  double max_dt_{0.2};
  double lateral_velocity_tolerance_{2.0e-2};
  double pose_covariance_xy_{1.0e-3};
  double pose_covariance_yaw_{1.0e-3};
  double twist_covariance_x_{1.0e-3};
  double twist_covariance_y_{1.0e-3};
  double twist_covariance_yaw_{1.0e-3};
  double unobserved_variance_{1.0e6};

  bool initialized_{false};
  ros::Time last_stamp_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double left_wheel_position_{0.0};
  double right_wheel_position_{0.0};
};

}  // namespace diff_tracked_base

int main(int argc, char** argv) {
  ros::init(argc, argv, "virtual_encoder_odometry");
  diff_tracked_base::VirtualEncoderOdometry node;
  if (!node.Initialize()) {
    return 1;
  }
  ros::spin();
  return 0;
}
