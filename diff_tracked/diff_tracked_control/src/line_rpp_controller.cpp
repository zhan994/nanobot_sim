#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <string>
#include <utility>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Trigger.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace diff_tracked_control {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kCurvatureEpsilon = 1.0e-8;
constexpr double kGoalLateralTolerance = 0.20;

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

double Approach(double value, double target, double max_delta) {
  return value + Clamp(target - value, -max_delta, max_delta);
}

double NormalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double YawFromQuaternion(const geometry_msgs::Quaternion& quaternion) {
  const double siny_cosp =
      2.0 * (quaternion.w * quaternion.z +
             quaternion.x * quaternion.y);
  const double cosy_cosp =
      1.0 - 2.0 * (quaternion.y * quaternion.y +
                   quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

class RegulatedPurePursuitController {
 public:
  RegulatedPurePursuitController()
      : private_nh_("~"), tf_listener_(tf_buffer_) {}

  bool Initialize() {
    LoadParameters();
    if (!ValidateParameters()) {
      return false;
    }

    cmd_vel_publisher_ =
        nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    planned_path_publisher_ =
        nh_.advertise<nav_msgs::Path>(planned_path_topic_, 1, true);
    actual_path_publisher_ =
        nh_.advertise<nav_msgs::Path>(actual_path_topic_, 1, true);
    initial_pose_publisher_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            initial_pose_topic_, 1);

    odom_subscriber_ = nh_.subscribe(
        odom_topic_, 1,
        &RegulatedPurePursuitController::OdomCallback, this);
    scan_subscriber_ = nh_.subscribe(
        scan_topic_, 1,
        &RegulatedPurePursuitController::ScanCallback, this);
    confirm_origin_service_ = nh_.advertiseService(
        confirm_origin_service_name_,
        &RegulatedPurePursuitController::ConfirmMapOrigin, this);

    control_enabled_ = !require_origin_confirmation_;
    if (!control_enabled_) {
      ROS_INFO("RPP disabled; waiting for service %s",
               confirm_origin_service_name_.c_str());
    }

    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / control_rate_),
        &RegulatedPurePursuitController::ControlCallback, this);
    return true;
  }

 private:
  struct TrackingState {
    double along = 0.0;
    double lateral = 0.0;
    double remaining = 0.0;
    double goal_distance = 0.0;
    double lookahead = 0.0;
    double curvature = 0.0;
  };

  void LoadParameters() {
    private_nh_.param("odom_topic", odom_topic_, std::string("/odom"));
    private_nh_.param("cmd_vel_topic", cmd_vel_topic_,
                      std::string("/cmd_vel"));
    private_nh_.param("planned_path_topic", planned_path_topic_,
                      std::string("/planned_path"));
    private_nh_.param("actual_path_topic", actual_path_topic_,
                      std::string("/actual_path"));
    private_nh_.param("initial_pose_topic", initial_pose_topic_,
                      std::string("/initialpose"));
    private_nh_.param("scan_topic", scan_topic_, std::string("/scan"));
    private_nh_.param("confirm_origin_service",
                      confirm_origin_service_name_,
                      std::string("/confirm_map_origin"));
    private_nh_.param("map_frame", map_frame_, std::string("map"));
    private_nh_.param("base_frame", base_frame_, std::string("base_link"));

    private_nh_.param("require_origin_confirmation",
                      require_origin_confirmation_, false);
    private_nh_.param("origin_pose_x", origin_pose_x_, 0.0);
    private_nh_.param("origin_pose_y", origin_pose_y_, 0.0);
    private_nh_.param("origin_pose_yaw", origin_pose_yaw_, 0.0);
    private_nh_.param("origin_position_covariance",
                      origin_position_covariance_, 0.0025);
    private_nh_.param("origin_yaw_covariance",
                      origin_yaw_covariance_, 0.0012);
    private_nh_.param("localization_pose_timeout",
                      localization_pose_timeout_, 10.0);
    private_nh_.param("origin_position_tolerance",
                      origin_position_tolerance_, 0.30);
    private_nh_.param("origin_yaw_tolerance",
                      origin_yaw_tolerance_, 0.35);
    private_nh_.param("localization_stable_position_tolerance",
                      localization_stable_position_tolerance_, 0.02);
    private_nh_.param("localization_stable_yaw_tolerance",
                      localization_stable_yaw_tolerance_, 0.02);
    private_nh_.param("localization_stable_duration",
                      localization_stable_duration_, 1.0);

    private_nh_.param("transform_timeout", transform_timeout_, 0.05);
    private_nh_.param("target_distance", target_distance_, 5.0);
    private_nh_.param("cruise_speed", desired_speed_, 0.5);
    private_nh_.param("initial_heading_deg", heading_offset_deg_, 0.0);
    private_nh_.param("min_linear_speed", min_linear_speed_, 0.05);
    private_nh_.param("max_angular_speed", max_angular_speed_, 1.0);
    private_nh_.param("distance_tolerance", distance_tolerance_, 0.03);
    private_nh_.param("control_rate", control_rate_, 20.0);
    private_nh_.param("startup_delay", startup_delay_, 2.0);

    private_nh_.param("lookahead_distance", lookahead_distance_, 0.5);
    private_nh_.param("min_lookahead_distance",
                      min_lookahead_distance_, 0.25);
    private_nh_.param("max_lookahead_distance",
                      max_lookahead_distance_, 1.0);
    private_nh_.param("lookahead_time", lookahead_time_, 1.0);
    private_nh_.param("regulated_min_radius", regulated_min_radius_, 0.9);
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
    if (odom_topic_.empty() || cmd_vel_topic_.empty() ||
        planned_path_topic_.empty() || actual_path_topic_.empty() ||
        initial_pose_topic_.empty() || scan_topic_.empty() ||
        confirm_origin_service_name_.empty() || map_frame_.empty() ||
        base_frame_.empty()) {
      ROS_ERROR("frame, topic and service names must not be empty");
      return false;
    }

    if (!std::isfinite(heading_offset_deg_) ||
        !std::isfinite(origin_pose_x_) ||
        !std::isfinite(origin_pose_y_) ||
        !std::isfinite(origin_pose_yaw_)) {
      ROS_ERROR("heading and origin pose parameters must be finite");
      return false;
    }

    if (!CheckPositive({
            {"target_distance", target_distance_},
            {"cruise_speed", desired_speed_},
            {"max_angular_speed", max_angular_speed_},
            {"distance_tolerance", distance_tolerance_},
            {"control_rate", control_rate_},
            {"lookahead_distance", lookahead_distance_},
            {"min_lookahead_distance", min_lookahead_distance_},
            {"max_lookahead_distance", max_lookahead_distance_},
            {"regulated_min_radius", regulated_min_radius_},
            {"max_lateral_accel", max_lateral_accel_},
            {"approach_distance", approach_distance_},
            {"max_linear_accel", max_linear_accel_},
            {"max_linear_decel", max_linear_decel_},
            {"max_angular_accel", max_angular_accel_},
            {"odom_timeout", odom_timeout_},
            {"planned_path_resolution", planned_path_resolution_},
            {"actual_path_min_distance", actual_path_min_distance_},
            {"localization_pose_timeout", localization_pose_timeout_},
            {"origin_position_tolerance", origin_position_tolerance_},
            {"origin_yaw_tolerance", origin_yaw_tolerance_},
            {"localization_stable_position_tolerance",
             localization_stable_position_tolerance_},
            {"localization_stable_yaw_tolerance",
             localization_stable_yaw_tolerance_},
            {"localization_stable_duration",
             localization_stable_duration_}})) {
      return false;
    }

    if (transform_timeout_ < 0.0 || startup_delay_ < 0.0 ||
        lookahead_time_ < 0.0 || min_linear_speed_ < 0.0 ||
        origin_position_covariance_ < 0.0 ||
        origin_yaw_covariance_ < 0.0) {
      ROS_ERROR("timeout, delay, speed and covariance limits must be non-negative");
      return false;
    }
    if (min_lookahead_distance_ > max_lookahead_distance_) {
      ROS_ERROR("~min_lookahead_distance must not exceed the maximum");
      return false;
    }
    if (actual_path_max_poses_ <= 0) {
      ROS_ERROR("~actual_path_max_poses must be positive");
      return false;
    }
    return true;
  }

  bool CheckPositive(
      std::initializer_list<std::pair<const char*, double>> values) const {
    for (const auto& item : values) {
      if (item.second <= 0.0) {
        ROS_ERROR("~%s must be positive", item.first);
        return false;
      }
    }
    return true;
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    latest_odom_ = *message;
    has_latest_odom_ = true;
  }

  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& message) {
    if (!waiting_for_localization_ || has_scan_after_request_) {
      return;
    }
    if (message->header.stamp.isZero() ||
        message->header.stamp + ros::Duration(0.5) >=
            initial_pose_sent_stamp_) {
      has_scan_after_request_ = true;
    }
  }

  bool ConfirmMapOrigin(std_srvs::Trigger::Request&,
                        std_srvs::Trigger::Response& response) {
    if (initial_pose_publisher_.getNumSubscribers() == 0 ||
        scan_subscriber_.getNumPublishers() == 0) {
      response.success = false;
      response.message = "slam_toolbox or laser scan is not connected";
      return true;
    }

    control_enabled_ = false;
    waiting_for_localization_ = true;
    has_scan_after_request_ = false;
    localization_stability_started_ = false;
    initial_pose_sent_stamp_ = ros::Time::now();
    localization_request_wall_time_ = ros::WallTime::now();
    ResetTrackingState();
    PublishStop();
    PublishInitialPose();

    response.success = true;
    response.message = "initial pose published; waiting for localization";
    ROS_INFO("Confirmed map origin at (%.3f, %.3f, %.3f rad)",
             origin_pose_x_, origin_pose_y_, origin_pose_yaw_);
    return true;
  }

  void PublishInitialPose() {
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = initial_pose_sent_stamp_;
    pose.pose.pose.position.x = origin_pose_x_;
    pose.pose.pose.position.y = origin_pose_y_;
    pose.pose.pose.orientation.z = std::sin(0.5 * origin_pose_yaw_);
    pose.pose.pose.orientation.w = std::cos(0.5 * origin_pose_yaw_);
    pose.pose.covariance[0] = origin_position_covariance_;
    pose.pose.covariance[7] = origin_position_covariance_;
    pose.pose.covariance[35] = origin_yaw_covariance_;
    initial_pose_publisher_.publish(pose);
  }

  void ResetTrackingState() {
    has_start_pose_ = false;
    startup_delay_started_ = false;
    finished_ = false;
    has_last_actual_position_ = false;
    actual_path_.poses.clear();
    last_linear_command_ = 0.0;
    last_angular_command_ = 0.0;
  }

  void UpdateLocalizationGate() {
    if (!waiting_for_localization_) {
      return;
    }

    const double elapsed =
        (ros::WallTime::now() - localization_request_wall_time_).toSec();
    if (elapsed > localization_pose_timeout_) {
      waiting_for_localization_ = false;
      ROS_ERROR("Localization timed out after %.1f s; RPP remains disabled",
                localization_pose_timeout_);
      return;
    }
    if (!has_scan_after_request_) {
      return;
    }

    geometry_msgs::Pose robot_pose;
    ros::Time stamp;
    if (!LookupRobotPose(robot_pose, stamp)) {
      return;
    }

    const double position_error =
        std::hypot(robot_pose.position.x - origin_pose_x_,
                   robot_pose.position.y - origin_pose_y_);
    const double yaw_error = std::abs(NormalizeAngle(
        YawFromQuaternion(robot_pose.orientation) - origin_pose_yaw_));
    if (position_error > origin_position_tolerance_ ||
        yaw_error > origin_yaw_tolerance_) {
      localization_stability_started_ = false;
      ROS_WARN_THROTTLE(
          1.0,
          "Waiting for localization near origin: position %.3f m, yaw %.3f rad",
          position_error, yaw_error);
      return;
    }

    if (!localization_stability_started_) {
      localization_stability_started_ = true;
      localization_stability_wall_time_ = ros::WallTime::now();
      localization_anchor_x_ = robot_pose.position.x;
      localization_anchor_y_ = robot_pose.position.y;
      localization_anchor_yaw_ = YawFromQuaternion(robot_pose.orientation);
      return;
    }

    const double position_delta =
        std::hypot(robot_pose.position.x - localization_anchor_x_,
                   robot_pose.position.y - localization_anchor_y_);
    const double yaw_delta = std::abs(NormalizeAngle(
        YawFromQuaternion(robot_pose.orientation) -
        localization_anchor_yaw_));
    if (position_delta > localization_stable_position_tolerance_ ||
        yaw_delta > localization_stable_yaw_tolerance_) {
      localization_stability_started_ = false;
      return;
    }

    const double stable_time =
        (ros::WallTime::now() - localization_stability_wall_time_).toSec();
    if (stable_time < localization_stable_duration_) {
      return;
    }

    waiting_for_localization_ = false;
    ResetTrackingState();
    control_enabled_ = true;
    ROS_INFO("Localization stable; RPP starts after %.1f s", startup_delay_);
  }

  bool LookupRobotPose(geometry_msgs::Pose& robot_pose,
                       ros::Time& transform_stamp) {
    try {
      const geometry_msgs::TransformStamped transform =
          tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0),
                                     ros::Duration(transform_timeout_));
      robot_pose.position.x = transform.transform.translation.x;
      robot_pose.position.y = transform.transform.translation.y;
      robot_pose.position.z = transform.transform.translation.z;
      robot_pose.orientation = transform.transform.rotation;
      transform_stamp = transform.header.stamp;
      return true;
    } catch (const tf2::TransformException& exception) {
      ROS_WARN_THROTTLE(1.0, "Cannot transform %s -> %s: %s",
                        map_frame_.c_str(), base_frame_.c_str(),
                        exception.what());
      return false;
    }
  }

  bool StartupDelayElapsed() {
    if (!startup_delay_started_) {
      startup_wall_time_ = ros::WallTime::now();
      startup_delay_started_ = true;
    }
    const double elapsed =
        (ros::WallTime::now() - startup_wall_time_).toSec();
    if (elapsed >= startup_delay_) {
      return true;
    }
    ROS_INFO_THROTTLE(1.0, "Controller startup delay: %.1f s remaining",
                      startup_delay_ - elapsed);
    return false;
  }

  void InitializeStartPose(const geometry_msgs::Pose& robot_pose,
                           const ros::Time& stamp) {
    start_x_ = robot_pose.position.x;
    start_y_ = robot_pose.position.y;
    start_yaw_ = NormalizeAngle(
        YawFromQuaternion(robot_pose.orientation) +
        heading_offset_deg_ * kPi / 180.0);
    has_start_pose_ = true;
    PublishPlannedPath(stamp);
    ROS_INFO("RPP start at (%.3f, %.3f), heading %.1f deg, offset %.1f deg",
             start_x_, start_y_, start_yaw_ * 180.0 / kPi,
             heading_offset_deg_);
  }

  void PublishPlannedPath(const ros::Time& stamp) {
    const int count = std::max(
        1, static_cast<int>(
               std::ceil(target_distance_ / planned_path_resolution_)));
    nav_msgs::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = stamp;
    path.poses.reserve(count + 1);

    for (int i = 0; i <= count; ++i) {
      const double distance =
          target_distance_ * static_cast<double>(i) / count;
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = start_x_ + distance * std::cos(start_yaw_);
      pose.pose.position.y = start_y_ + distance * std::sin(start_yaw_);
      pose.pose.orientation.z = std::sin(0.5 * start_yaw_);
      pose.pose.orientation.w = std::cos(0.5 * start_yaw_);
      path.poses.push_back(pose);
    }
    planned_path_publisher_.publish(path);
    ROS_INFO("Published %.1f m planned path with %zu poses",
             target_distance_, path.poses.size());
  }

  void RecordActualPath(const geometry_msgs::Pose& robot_pose,
                        const ros::Time& stamp) {
    const geometry_msgs::Point& point = robot_pose.position;
    if (has_last_actual_position_ &&
        std::hypot(point.x - last_actual_x_,
                   point.y - last_actual_y_) < actual_path_min_distance_) {
      return;
    }

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = stamp;
    pose.pose = robot_pose;
    actual_path_.header = pose.header;
    actual_path_.poses.push_back(pose);
    if (actual_path_.poses.size() >
        static_cast<std::size_t>(actual_path_max_poses_)) {
      actual_path_.poses.erase(actual_path_.poses.begin());
    }

    last_actual_x_ = point.x;
    last_actual_y_ = point.y;
    has_last_actual_position_ = true;
    actual_path_publisher_.publish(actual_path_);
  }

  bool IsOdometryFresh() const {
    const ros::Time stamp = latest_odom_.header.stamp;
    if (stamp.isZero() ||
        (ros::Time::now() - stamp).toSec() <= odom_timeout_) {
      return true;
    }
    ROS_WARN_THROTTLE(2.0, "Odometry is stale; commanding stop");
    return false;
  }

  TrackingState ComputeTrackingState(
      const geometry_msgs::Pose& robot_pose) const {
    TrackingState state;
    const double robot_x = robot_pose.position.x;
    const double robot_y = robot_pose.position.y;
    const double robot_yaw = YawFromQuaternion(robot_pose.orientation);
    const double dx = robot_x - start_x_;
    const double dy = robot_y - start_y_;

    state.along = std::cos(start_yaw_) * dx + std::sin(start_yaw_) * dy;
    state.lateral =
        -std::sin(start_yaw_) * dx + std::cos(start_yaw_) * dy;
    state.remaining = target_distance_ - state.along;

    const double goal_x =
        start_x_ + target_distance_ * std::cos(start_yaw_);
    const double goal_y =
        start_y_ + target_distance_ * std::sin(start_yaw_);
    state.goal_distance =
        std::hypot(goal_x - robot_x, goal_y - robot_y);

    const double measured_speed =
        std::abs(latest_odom_.twist.twist.linear.x);
    state.lookahead = Clamp(
        lookahead_distance_ + lookahead_time_ * measured_speed,
        min_lookahead_distance_, max_lookahead_distance_);
    const double carrot_distance =
        Clamp(state.along + state.lookahead, 0.0, target_distance_);
    const double carrot_x =
        start_x_ + carrot_distance * std::cos(start_yaw_);
    const double carrot_y =
        start_y_ + carrot_distance * std::sin(start_yaw_);
    const double carrot_dx = carrot_x - robot_x;
    const double carrot_dy = carrot_y - robot_y;
    const double local_x =
        std::cos(robot_yaw) * carrot_dx + std::sin(robot_yaw) * carrot_dy;
    const double local_y =
        -std::sin(robot_yaw) * carrot_dx + std::cos(robot_yaw) * carrot_dy;
    const double distance_squared = local_x * local_x + local_y * local_y;
    if (distance_squared > kCurvatureEpsilon) {
      state.curvature = 2.0 * local_y / distance_squared;
    }
    return state;
  }

  double ComputeTargetSpeed(const TrackingState& state) const {
    double speed = desired_speed_;
    const double curvature = std::abs(state.curvature);
    if (curvature > kCurvatureEpsilon) {
      const double radius = 1.0 / curvature;
      if (radius < regulated_min_radius_) {
        speed = radius / regulated_min_radius_;
      }
      speed = std::min(speed, std::sqrt(max_lateral_accel_ / curvature));
    }
    if (state.remaining < approach_distance_) {
      speed *= state.remaining / approach_distance_;
    }
    speed = Clamp(speed, min_linear_speed_, desired_speed_);
    if (curvature > kCurvatureEpsilon) {
      speed = std::min(speed, max_angular_speed_ / curvature);
    }
    return speed;
  }

  void PublishCommand(const TrackingState& state, double period) {
    const double target_speed = ComputeTargetSpeed(state);
    const double acceleration =
        target_speed >= last_linear_command_
            ? max_linear_accel_
            : max_linear_decel_;
    const double linear =
        Approach(last_linear_command_, target_speed, acceleration * period);
    const double target_angular =
        Clamp(linear * state.curvature,
              -max_angular_speed_, max_angular_speed_);
    const double angular =
        Approach(last_angular_command_, target_angular,
                 max_angular_accel_ * period);

    geometry_msgs::Twist command;
    command.linear.x = linear;
    command.angular.z = angular;
    cmd_vel_publisher_.publish(command);
    last_linear_command_ = linear;
    last_angular_command_ = angular;

    ROS_INFO_THROTTLE(
        1.0,
        "RPP along %.2f/%.2f m, lateral %.3f m, lookahead %.2f m, "
        "curvature %.3f 1/m, cmd (%.2f m/s, %.2f rad/s)",
        state.along, target_distance_, state.lateral, state.lookahead,
        state.curvature, linear, angular);
  }

  void ControlCallback(const ros::TimerEvent& event) {
    UpdateLocalizationGate();
    if (!control_enabled_) {
      PublishStop();
      return;
    }
    if (!has_latest_odom_) {
      ROS_WARN_THROTTLE(2.0, "waiting for odometry on %s",
                        odom_topic_.c_str());
      PublishStop();
      return;
    }

    geometry_msgs::Pose robot_pose;
    ros::Time stamp;
    if (!LookupRobotPose(robot_pose, stamp)) {
      PublishStop();
      return;
    }
    if (stamp.isZero()) {
      stamp = ros::Time::now();
    }
    if (!StartupDelayElapsed()) {
      PublishStop();
      return;
    }
    if (!has_start_pose_) {
      InitializeStartPose(robot_pose, stamp);
    }
    if (!IsOdometryFresh() || finished_) {
      PublishStop();
      return;
    }

    RecordActualPath(robot_pose, stamp);
    const double period = event.last_real.isZero()
                              ? 1.0 / control_rate_
                              : (event.current_real - event.last_real).toSec();
    if (period <= 0.0) {
      return;
    }

    const TrackingState state = ComputeTrackingState(robot_pose);
    if (state.goal_distance <= distance_tolerance_ &&
        std::abs(state.lateral) <= kGoalLateralTolerance) {
      finished_ = true;
      PublishStop();
      ROS_INFO("Target reached: distance %.3f m, along %.3f m, lateral %.3f m",
               state.goal_distance, state.along, state.lateral);
      return;
    }
    PublishCommand(state, period);
  }

  void PublishStop() {
    last_linear_command_ = 0.0;
    last_angular_command_ = 0.0;
    cmd_vel_publisher_.publish(geometry_msgs::Twist());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher cmd_vel_publisher_;
  ros::Publisher planned_path_publisher_;
  ros::Publisher actual_path_publisher_;
  ros::Publisher initial_pose_publisher_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber scan_subscriber_;
  ros::ServiceServer confirm_origin_service_;
  ros::Timer control_timer_;

  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string planned_path_topic_;
  std::string actual_path_topic_;
  std::string initial_pose_topic_;
  std::string scan_topic_;
  std::string confirm_origin_service_name_;
  std::string map_frame_;
  std::string base_frame_;

  bool require_origin_confirmation_ = false;
  double origin_pose_x_ = 0.0;
  double origin_pose_y_ = 0.0;
  double origin_pose_yaw_ = 0.0;
  double origin_position_covariance_ = 0.0025;
  double origin_yaw_covariance_ = 0.0012;
  double localization_pose_timeout_ = 10.0;
  double origin_position_tolerance_ = 0.30;
  double origin_yaw_tolerance_ = 0.35;
  double localization_stable_position_tolerance_ = 0.02;
  double localization_stable_yaw_tolerance_ = 0.02;
  double localization_stable_duration_ = 1.0;
  double transform_timeout_ = 0.05;

  double target_distance_ = 5.0;
  double desired_speed_ = 0.5;
  double heading_offset_deg_ = 0.0;
  double min_linear_speed_ = 0.05;
  double max_angular_speed_ = 1.0;
  double distance_tolerance_ = 0.03;
  double control_rate_ = 20.0;
  double startup_delay_ = 2.0;
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

  bool has_latest_odom_ = false;
  bool startup_delay_started_ = false;
  bool finished_ = false;
  bool control_enabled_ = true;
  bool waiting_for_localization_ = false;
  bool has_scan_after_request_ = false;
  bool localization_stability_started_ = false;
  bool has_start_pose_ = false;
  bool has_last_actual_position_ = false;

  ros::WallTime startup_wall_time_;
  ros::WallTime localization_request_wall_time_;
  ros::WallTime localization_stability_wall_time_;
  ros::Time initial_pose_sent_stamp_;
  double localization_anchor_x_ = 0.0;
  double localization_anchor_y_ = 0.0;
  double localization_anchor_yaw_ = 0.0;
  double start_x_ = 0.0;
  double start_y_ = 0.0;
  double start_yaw_ = 0.0;
  double last_actual_x_ = 0.0;
  double last_actual_y_ = 0.0;
  double last_linear_command_ = 0.0;
  double last_angular_command_ = 0.0;

  nav_msgs::Odometry latest_odom_;
  nav_msgs::Path actual_path_;
};

}  // namespace diff_tracked_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "regulated_pure_pursuit_controller");
  diff_tracked_control::RegulatedPurePursuitController controller;
  if (!controller.Initialize()) {
    ROS_FATAL("Failed to initialize RPP controller");
    return 1;
  }
  ros::spin();
  return 0;
}
