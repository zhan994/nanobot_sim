#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

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

constexpr double kEpsilon = 1.0e-8;

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

double Distance(const geometry_msgs::Point& a,
                const geometry_msgs::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace

class PathRppController {
 public:
  PathRppController() : private_nh_("~"), tf_listener_(tf_buffer_) {}

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
        odom_topic_, 1, &PathRppController::OdomCallback, this);
    path_subscriber_ = nh_.subscribe(
        path_topic_, 1, &PathRppController::PathCallback, this);
    scan_subscriber_ = nh_.subscribe(
        scan_topic_, 1, &PathRppController::ScanCallback, this);
    confirm_origin_service_ = nh_.advertiseService(
        confirm_origin_service_name_,
        &PathRppController::ConfirmMapOrigin, this);

    control_enabled_ = !require_origin_confirmation_;
    if (!control_enabled_) {
      ROS_INFO("Path RPP disabled; waiting for service %s",
               confirm_origin_service_name_.c_str());
    }

    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / control_rate_),
        &PathRppController::ControlCallback, this);
    return true;
  }

 private:
  struct TrackingState {
    double progress = 0.0;
    double remaining = 0.0;
    double lateral_error = 0.0;
    double goal_distance = 0.0;
    double lookahead = 0.0;
    double target_heading = 0.0;
    double curvature = 0.0;
    double path_curvature = 0.0;
    double speed_limit = 0.0;
  };

  void LoadParameters() {
    private_nh_.param("odom_topic", odom_topic_, std::string("/odom"));
    private_nh_.param("path_topic", path_topic_,
                      std::string("/coverage_path"));
    private_nh_.param("cmd_vel_topic", cmd_vel_topic_,
                      std::string("/cmd_vel"));
    private_nh_.param("planned_path_topic", planned_path_topic_,
                      std::string("/planned_path"));
    private_nh_.param("actual_path_topic", actual_path_topic_,
                      std::string("/actual_path"));
    private_nh_.param("initial_pose_topic", initial_pose_topic_,
                      std::string("/initialpose"));
    private_nh_.param("scan_topic", scan_topic_, std::string("/scan_2d"));
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
                      localization_pose_timeout_, 15.0);
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
    private_nh_.param("control_rate", control_rate_, 20.0);
    private_nh_.param("startup_delay", startup_delay_, 2.0);
    private_nh_.param("odom_timeout", odom_timeout_, 1.0);
    private_nh_.param("distance_tolerance", distance_tolerance_, 0.10);

    private_nh_.param("cruise_speed", desired_speed_, 0.5);
    private_nh_.param("min_linear_speed", min_linear_speed_, 0.05);
    private_nh_.param("max_angular_speed", max_angular_speed_, 0.8);
    private_nh_.param("max_linear_accel", max_linear_accel_, 0.4);
    private_nh_.param("max_linear_decel", max_linear_decel_, 0.6);
    private_nh_.param("max_angular_accel", max_angular_accel_, 1.0);
    private_nh_.param("regulated_min_radius", regulated_min_radius_, 1.0);
    private_nh_.param("max_lateral_accel", max_lateral_accel_, 0.5);
    private_nh_.param("approach_distance", approach_distance_, 2.0);

    private_nh_.param("lookahead_distance", lookahead_distance_, 0.6);
    private_nh_.param("min_lookahead_distance",
                      min_lookahead_distance_, 0.4);
    private_nh_.param("max_lookahead_distance",
                      max_lookahead_distance_, 1.2);
    private_nh_.param("lookahead_time", lookahead_time_, 1.0);
    private_nh_.param("curve_lookahead_gain", curve_lookahead_gain_, 2.0);
    private_nh_.param("curvature_sample_distance",
                      curvature_sample_distance_, 0.4);
    private_nh_.param("curve_speed_safety_factor",
                      curve_speed_safety_factor_, 0.85);
    private_nh_.param("path_search_distance", path_search_distance_, 6.0);
    private_nh_.param("path_backtrack_distance",
                      path_backtrack_distance_, 0.5);
    private_nh_.param("start_from_nearest", start_from_nearest_, false);

    private_nh_.param("rotate_in_place", rotate_in_place_, true);
    private_nh_.param("rotate_in_place_threshold",
                      rotate_in_place_threshold_, 1.0);
    private_nh_.param("rotate_in_place_speed",
                      rotate_in_place_speed_, 0.4);
    private_nh_.param("actual_path_min_distance",
                      actual_path_min_distance_, 0.05);
    private_nh_.param("actual_path_max_poses",
                      actual_path_max_poses_, 5000);
  }

  bool ValidateParameters() const {
    if (odom_topic_.empty() || path_topic_.empty() ||
        cmd_vel_topic_.empty() || planned_path_topic_.empty() ||
        actual_path_topic_.empty() || initial_pose_topic_.empty() ||
        scan_topic_.empty() || confirm_origin_service_name_.empty() ||
        map_frame_.empty() || base_frame_.empty()) {
      ROS_ERROR("frame, topic and service names must not be empty");
      return false;
    }
    if (control_rate_ <= 0.0 || desired_speed_ <= 0.0 ||
        max_angular_speed_ <= 0.0 || max_linear_accel_ <= 0.0 ||
        max_linear_decel_ <= 0.0 || max_angular_accel_ <= 0.0 ||
        distance_tolerance_ <= 0.0 || lookahead_distance_ <= 0.0 ||
        min_lookahead_distance_ <= 0.0 ||
        max_lookahead_distance_ <= 0.0 || path_search_distance_ <= 0.0 ||
        regulated_min_radius_ <= 0.0 || max_lateral_accel_ <= 0.0 ||
        approach_distance_ <= 0.0 || localization_pose_timeout_ <= 0.0 ||
        localization_stable_duration_ <= 0.0 ||
        curve_speed_safety_factor_ <= 0.0 ||
        curve_speed_safety_factor_ > 1.0) {
      ROS_ERROR("positive path RPP parameters must be greater than zero");
      return false;
    }
    if (min_linear_speed_ < 0.0 || transform_timeout_ < 0.0 ||
        startup_delay_ < 0.0 || odom_timeout_ < 0.0 ||
        lookahead_time_ < 0.0 || curve_lookahead_gain_ < 0.0 ||
        path_backtrack_distance_ < 0.0 ||
        rotate_in_place_threshold_ < 0.0 || rotate_in_place_speed_ < 0.0 ||
        actual_path_min_distance_ < 0.0 || actual_path_max_poses_ <= 0) {
      ROS_ERROR("non-negative path RPP parameters contain an invalid value");
      return false;
    }
    if (rotate_in_place_ && rotate_in_place_speed_ <= 0.0) {
      ROS_ERROR("~rotate_in_place_speed must be positive when rotation is enabled");
      return false;
    }
    if (min_lookahead_distance_ > max_lookahead_distance_) {
      ROS_ERROR("minimum lookahead must not exceed maximum lookahead");
      return false;
    }
    if (curvature_sample_distance_ <= 0.0) {
      ROS_ERROR("~curvature_sample_distance must be positive");
      return false;
    }
    return true;
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    latest_odom_ = *message;
    has_latest_odom_ = true;
  }

  void PathCallback(const nav_msgs::Path::ConstPtr& message) {
    if (message->poses.size() < 2) {
      ROS_WARN("Ignoring coverage path with fewer than two poses");
      return;
    }

    const std::string frame = message->header.frame_id.empty()
                                  ? message->poses.front().header.frame_id
                                  : message->header.frame_id;
    if (frame.empty()) {
      ROS_ERROR("Ignoring coverage path without a frame_id");
      return;
    }

    std::vector<geometry_msgs::Point> points;
    points.reserve(message->poses.size());
    for (const auto& pose : message->poses) {
      const geometry_msgs::Point& point = pose.pose.position;
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        ROS_ERROR("Ignoring coverage path containing a non-finite point");
        return;
      }
      if (points.empty() || Distance(points.back(), point) > 1.0e-4) {
        points.push_back(point);
      }
    }
    if (points.size() < 2) {
      ROS_WARN("Ignoring coverage path without usable segments");
      return;
    }

    std::vector<double> lengths(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
      lengths[i] = lengths[i - 1] + Distance(points[i - 1], points[i]);
    }
    if (lengths.back() <= kEpsilon) {
      ROS_WARN("Ignoring zero-length coverage path");
      return;
    }

    path_points_.swap(points);
    cumulative_lengths_.swap(lengths);
    BuildSpeedProfile();
    path_frame_ = frame;
    planned_path_publisher_.publish(*message);
    ResetExecutionState();
    has_path_ = true;
    PublishStop();
    ROS_INFO("Received coverage path: %zu points, %.2f m, frame %s",
             path_points_.size(), cumulative_lengths_.back(),
             path_frame_.c_str());
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
    ResetExecutionState();
    PublishStop();

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

    response.success = true;
    response.message = "initial pose published; waiting for localization";
    return true;
  }

  void ResetExecutionState() {
    startup_delay_started_ = false;
    progress_initialized_ = false;
    path_progress_ = 0.0;
    finished_ = false;
    actual_path_.poses.clear();
    has_last_actual_position_ = false;
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
      ROS_ERROR("Localization timed out; path RPP remains disabled");
      return;
    }
    if (!has_scan_after_request_) {
      return;
    }

    geometry_msgs::Pose robot_pose;
    ros::Time stamp;
    if (!LookupRobotPose(map_frame_, robot_pose, stamp)) {
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
      ROS_WARN_THROTTLE(1.0,
                        "Waiting for localization: position %.3f m, yaw %.3f rad",
                        position_error, yaw_error);
      return;
    }

    const double robot_yaw = YawFromQuaternion(robot_pose.orientation);
    if (!localization_stability_started_) {
      localization_stability_started_ = true;
      localization_stability_wall_time_ = ros::WallTime::now();
      localization_anchor_x_ = robot_pose.position.x;
      localization_anchor_y_ = robot_pose.position.y;
      localization_anchor_yaw_ = robot_yaw;
      return;
    }
    const double position_delta =
        std::hypot(robot_pose.position.x - localization_anchor_x_,
                   robot_pose.position.y - localization_anchor_y_);
    const double yaw_delta =
        std::abs(NormalizeAngle(robot_yaw - localization_anchor_yaw_));
    if (position_delta > localization_stable_position_tolerance_ ||
        yaw_delta > localization_stable_yaw_tolerance_) {
      localization_stability_started_ = false;
      return;
    }
    if ((ros::WallTime::now() - localization_stability_wall_time_).toSec() <
        localization_stable_duration_) {
      return;
    }

    waiting_for_localization_ = false;
    ResetExecutionState();
    control_enabled_ = true;
    ROS_INFO("Localization stable; path RPP starts after %.1f s",
             startup_delay_);
  }

  bool LookupRobotPose(const std::string& target_frame,
                       geometry_msgs::Pose& robot_pose,
                       ros::Time& stamp) {
    try {
      const geometry_msgs::TransformStamped transform =
          tf_buffer_.lookupTransform(target_frame, base_frame_, ros::Time(0),
                                     ros::Duration(transform_timeout_));
      robot_pose.position.x = transform.transform.translation.x;
      robot_pose.position.y = transform.transform.translation.y;
      robot_pose.position.z = transform.transform.translation.z;
      robot_pose.orientation = transform.transform.rotation;
      stamp = transform.header.stamp;
      return true;
    } catch (const tf2::TransformException& exception) {
      ROS_WARN_THROTTLE(1.0, "Cannot transform %s -> %s: %s",
                        target_frame.c_str(), base_frame_.c_str(),
                        exception.what());
      return false;
    }
  }

  bool ReadyToTrack() {
    if (!has_path_) {
      ROS_WARN_THROTTLE(2.0, "Waiting for path on %s", path_topic_.c_str());
      return false;
    }
    if (!has_latest_odom_) {
      ROS_WARN_THROTTLE(2.0, "Waiting for odometry on %s", odom_topic_.c_str());
      return false;
    }
    const ros::Time odom_stamp = latest_odom_.header.stamp;
    if (!odom_stamp.isZero() && odom_timeout_ > 0.0 &&
        (ros::Time::now() - odom_stamp).toSec() > odom_timeout_) {
      ROS_WARN_THROTTLE(2.0, "Odometry is stale; commanding stop");
      return false;
    }
    return true;
  }

  bool StartupDelayElapsed() {
    if (!startup_delay_started_) {
      startup_delay_started_ = true;
      startup_wall_time_ = ros::WallTime::now();
    }
    const double elapsed =
        (ros::WallTime::now() - startup_wall_time_).toSec();
    if (elapsed >= startup_delay_) {
      return true;
    }
    ROS_INFO_THROTTLE(1.0, "Path RPP startup delay: %.1f s remaining",
                      startup_delay_ - elapsed);
    return false;
  }

  double FindClosestProgress(const geometry_msgs::Point& robot,
                             double& lateral_error) {
    const double total_length = cumulative_lengths_.back();
    double lower = 0.0;
    double upper = total_length;
    if (progress_initialized_) {
      lower = std::max(0.0, path_progress_ - path_backtrack_distance_);
      upper = std::min(total_length,
                       path_progress_ + path_search_distance_);
    } else if (!start_from_nearest_) {
      upper = std::min(total_length, path_search_distance_);
    }

    double best_distance_squared = std::numeric_limits<double>::infinity();
    double best_progress = path_progress_;
    for (std::size_t i = 0; i + 1 < path_points_.size(); ++i) {
      const double segment_start = cumulative_lengths_[i];
      const double segment_end = cumulative_lengths_[i + 1];
      if (segment_end < lower || segment_start > upper) {
        continue;
      }

      const geometry_msgs::Point& a = path_points_[i];
      const geometry_msgs::Point& b = path_points_[i + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double length_squared = dx * dx + dy * dy;
      if (length_squared <= kEpsilon) {
        continue;
      }
      const double projection = Clamp(
          ((robot.x - a.x) * dx + (robot.y - a.y) * dy) /
              length_squared,
          0.0, 1.0);
      const double px = a.x + projection * dx;
      const double py = a.y + projection * dy;
      const double ex = robot.x - px;
      const double ey = robot.y - py;
      const double distance_squared = ex * ex + ey * ey;
      if (distance_squared < best_distance_squared) {
        best_distance_squared = distance_squared;
        best_progress = segment_start +
                        projection * (segment_end - segment_start);
      }
    }

    lateral_error = std::sqrt(best_distance_squared);
    if (!progress_initialized_) {
      progress_initialized_ = true;
      path_progress_ = best_progress;
    } else {
      path_progress_ = std::max(path_progress_, best_progress);
    }
    return path_progress_;
  }

  geometry_msgs::Point PointAt(double progress) const {
    progress = Clamp(progress, 0.0, cumulative_lengths_.back());
    const auto upper = std::upper_bound(
        cumulative_lengths_.begin(), cumulative_lengths_.end(), progress);
    std::size_t index = upper == cumulative_lengths_.begin()
                            ? 0
                            : static_cast<std::size_t>(
                                  upper - cumulative_lengths_.begin() - 1);
    if (index + 1 >= path_points_.size()) {
      return path_points_.back();
    }
    const double segment_length =
        cumulative_lengths_[index + 1] - cumulative_lengths_[index];
    const double ratio = segment_length <= kEpsilon
                             ? 0.0
                             : (progress - cumulative_lengths_[index]) /
                                   segment_length;
    geometry_msgs::Point point;
    point.x = path_points_[index].x +
              ratio * (path_points_[index + 1].x - path_points_[index].x);
    point.y = path_points_[index].y +
              ratio * (path_points_[index + 1].y - path_points_[index].y);
    point.z = path_points_[index].z +
              ratio * (path_points_[index + 1].z - path_points_[index].z);
    return point;
  }

  double PathCurvatureAt(double progress) const {
    const double total_length = cumulative_lengths_.back();
    const geometry_msgs::Point center = PointAt(progress);
    const geometry_msgs::Point before = PointAt(
        std::max(0.0, progress - curvature_sample_distance_));
    const geometry_msgs::Point after = PointAt(
        std::min(total_length, progress + curvature_sample_distance_));

    const double a = Distance(before, center);
    const double b = Distance(center, after);
    const double c = Distance(before, after);
    if (a <= kEpsilon || b <= kEpsilon || c <= kEpsilon) {
      return 0.0;
    }

    const double cross = std::abs(
        (center.x - before.x) * (after.y - before.y) -
        (center.y - before.y) * (after.x - before.x));
    return 2.0 * cross / (a * b * c);
  }

  double CurvatureSpeedLimit(double curvature) const {
    curvature = std::abs(curvature);
    if (curvature <= kEpsilon) {
      return desired_speed_;
    }

    double speed = desired_speed_;
    const double radius = 1.0 / curvature;
    if (radius < regulated_min_radius_) {
      speed = std::min(
          speed, desired_speed_ * radius / regulated_min_radius_);
    }
    speed = std::min(speed, std::sqrt(max_lateral_accel_ / curvature));
    speed = std::min(speed, max_angular_speed_ / curvature);
    if (speed < desired_speed_) {
      speed *= curve_speed_safety_factor_;
    }
    return Clamp(speed, 0.0, desired_speed_);
  }

  void BuildSpeedProfile() {
    path_speed_limits_.resize(path_points_.size());
    double minimum_curve_speed = desired_speed_;

    for (std::size_t i = 0; i < path_points_.size(); ++i) {
      path_speed_limits_[i] =
          CurvatureSpeedLimit(PathCurvatureAt(cumulative_lengths_[i]));
      minimum_curve_speed =
          std::min(minimum_curve_speed, path_speed_limits_[i]);
    }

    // Start braking on the preceding straight early enough to reach every
    // curve speed limit without exceeding max_linear_decel_.
    path_speed_limits_.back() = 0.0;
    for (std::size_t i = path_speed_limits_.size() - 1; i > 0; --i) {
      const double distance =
          cumulative_lengths_[i] - cumulative_lengths_[i - 1];
      const double braking_limit = std::sqrt(
          path_speed_limits_[i] * path_speed_limits_[i] +
          2.0 * max_linear_decel_ * distance);
      path_speed_limits_[i - 1] =
          std::min(path_speed_limits_[i - 1], braking_limit);
    }

    ROS_INFO("Path speed profile: cruise %.2f m/s, curve minimum %.2f m/s",
             desired_speed_, minimum_curve_speed);
  }

  double SpeedLimitAt(double progress) const {
    if (path_speed_limits_.empty()) {
      return desired_speed_;
    }
    progress = Clamp(progress, 0.0, cumulative_lengths_.back());
    const auto upper = std::upper_bound(
        cumulative_lengths_.begin(), cumulative_lengths_.end(), progress);
    std::size_t index = upper == cumulative_lengths_.begin()
                            ? 0
                            : static_cast<std::size_t>(
                                  upper - cumulative_lengths_.begin() - 1);
    if (index + 1 >= path_speed_limits_.size()) {
      return path_speed_limits_.back();
    }

    const double length =
        cumulative_lengths_[index + 1] - cumulative_lengths_[index];
    const double ratio = length <= kEpsilon
                             ? 0.0
                             : (progress - cumulative_lengths_[index]) /
                                   length;
    return path_speed_limits_[index] +
           ratio * (path_speed_limits_[index + 1] -
                    path_speed_limits_[index]);
  }

  TrackingState ComputeTrackingState(
      const geometry_msgs::Pose& robot_pose) {
    TrackingState state;
    const double robot_yaw = YawFromQuaternion(robot_pose.orientation);
    state.progress =
        FindClosestProgress(robot_pose.position, state.lateral_error);
    state.path_curvature = PathCurvatureAt(state.progress);

    const double nominal_lookahead = Clamp(
        lookahead_distance_ +
            lookahead_time_ * std::abs(latest_odom_.twist.twist.linear.x),
        min_lookahead_distance_, max_lookahead_distance_);
    state.lookahead = Clamp(
        nominal_lookahead /
            (1.0 + curve_lookahead_gain_ * state.path_curvature),
        min_lookahead_distance_, max_lookahead_distance_);
    state.remaining = cumulative_lengths_.back() - state.progress;

    const geometry_msgs::Point target =
        PointAt(state.progress + state.lookahead);
    const double dx = target.x - robot_pose.position.x;
    const double dy = target.y - robot_pose.position.y;
    const double local_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    const double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
    const double distance_squared = local_x * local_x + local_y * local_y;
    state.target_heading = std::atan2(local_y, local_x);
    if (distance_squared > kEpsilon) {
      state.curvature = 2.0 * local_y / distance_squared;
    }
    state.goal_distance = Distance(robot_pose.position, path_points_.back());
    state.speed_limit = SpeedLimitAt(state.progress);
    return state;
  }

  double ComputeTargetSpeed(const TrackingState& state) const {
    double speed = std::min(
        state.speed_limit, CurvatureSpeedLimit(state.curvature));
    if (state.remaining < approach_distance_) {
      speed *= Clamp(state.remaining / approach_distance_, 0.0, 1.0);
    }
    return Clamp(speed, min_linear_speed_, desired_speed_);
  }

  void PublishTrackingCommand(const TrackingState& state, double period) {
    const bool should_rotate =
        rotate_in_place_ &&
        std::abs(state.target_heading) > rotate_in_place_threshold_;
    if (should_rotate) {
      const double linear = Approach(
          last_linear_command_, 0.0, max_linear_decel_ * period);
      double target_angular = 0.0;
      if (std::abs(linear) < 0.02) {
        target_angular = std::copysign(
            std::min(rotate_in_place_speed_, max_angular_speed_),
            state.target_heading);
      }
      const double angular = Approach(
          last_angular_command_, target_angular,
          max_angular_accel_ * period);
      PublishVelocity(linear, angular);
      return;
    }

    const double target_speed = ComputeTargetSpeed(state);
    const double acceleration = target_speed >= last_linear_command_
                                    ? max_linear_accel_
                                    : max_linear_decel_;
    const double linear = Approach(
        last_linear_command_, target_speed, acceleration * period);
    const double target_angular = Clamp(
        linear * state.curvature,
        -max_angular_speed_, max_angular_speed_);
    const double angular = Approach(
        last_angular_command_, target_angular,
        max_angular_accel_ * period);
    PublishVelocity(linear, angular);
  }

  void PublishVelocity(double linear, double angular) {
    geometry_msgs::Twist command;
    command.linear.x = linear;
    command.angular.z = angular;
    cmd_vel_publisher_.publish(command);
    last_linear_command_ = linear;
    last_angular_command_ = angular;
  }

  void RecordActualPath(const geometry_msgs::Pose& robot_pose,
                        const ros::Time& stamp) {
    if (has_last_actual_position_ &&
        Distance(robot_pose.position, last_actual_position_) <
            actual_path_min_distance_) {
      return;
    }
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = path_frame_;
    pose.header.stamp = stamp;
    pose.pose = robot_pose;
    actual_path_.header = pose.header;
    actual_path_.poses.push_back(pose);
    if (actual_path_.poses.size() >
        static_cast<std::size_t>(actual_path_max_poses_)) {
      actual_path_.poses.erase(actual_path_.poses.begin());
    }
    last_actual_position_ = robot_pose.position;
    has_last_actual_position_ = true;
    actual_path_publisher_.publish(actual_path_);
  }

  void ControlCallback(const ros::TimerEvent& event) {
    UpdateLocalizationGate();
    if (!control_enabled_ || !ReadyToTrack()) {
      PublishStop();
      return;
    }

    geometry_msgs::Pose robot_pose;
    ros::Time stamp;
    if (!LookupRobotPose(path_frame_, robot_pose, stamp)) {
      PublishStop();
      return;
    }
    if (!StartupDelayElapsed()) {
      PublishStop();
      return;
    }
    if (finished_) {
      PublishStop();
      return;
    }

    const double period = event.last_real.isZero()
                              ? 1.0 / control_rate_
                              : (event.current_real - event.last_real).toSec();
    if (period <= 0.0) {
      return;
    }
    if (stamp.isZero()) {
      stamp = ros::Time::now();
    }
    RecordActualPath(robot_pose, stamp);

    const TrackingState state = ComputeTrackingState(robot_pose);
    if (state.goal_distance <= distance_tolerance_ &&
        state.remaining <= std::max(state.lookahead, distance_tolerance_)) {
      finished_ = true;
      PublishStop();
      ROS_INFO("Coverage path completed: %.2f m", cumulative_lengths_.back());
      return;
    }

    PublishTrackingCommand(state, period);
    ROS_INFO_THROTTLE(
        1.0,
        "Path RPP %.2f/%.2f m, lateral %.3f m, remaining %.2f m, "
        "heading %.2f rad, lookahead %.2f m, path curvature %.2f, "
        "speed limit %.2f, cmd (%.2f, %.2f)",
        state.progress, cumulative_lengths_.back(), state.lateral_error,
        state.remaining, state.target_heading, state.lookahead,
        state.path_curvature, state.speed_limit,
        last_linear_command_, last_angular_command_);
  }

  void PublishStop() {
    PublishVelocity(0.0, 0.0);
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
  ros::Subscriber path_subscriber_;
  ros::Subscriber scan_subscriber_;
  ros::ServiceServer confirm_origin_service_;
  ros::Timer control_timer_;

  std::string odom_topic_;
  std::string path_topic_;
  std::string cmd_vel_topic_;
  std::string planned_path_topic_;
  std::string actual_path_topic_;
  std::string initial_pose_topic_;
  std::string scan_topic_;
  std::string confirm_origin_service_name_;
  std::string map_frame_;
  std::string base_frame_;
  std::string path_frame_;

  bool require_origin_confirmation_ = false;
  double origin_pose_x_ = 0.0;
  double origin_pose_y_ = 0.0;
  double origin_pose_yaw_ = 0.0;
  double origin_position_covariance_ = 0.0025;
  double origin_yaw_covariance_ = 0.0012;
  double localization_pose_timeout_ = 15.0;
  double origin_position_tolerance_ = 0.30;
  double origin_yaw_tolerance_ = 0.35;
  double localization_stable_position_tolerance_ = 0.02;
  double localization_stable_yaw_tolerance_ = 0.02;
  double localization_stable_duration_ = 1.0;

  double transform_timeout_ = 0.05;
  double control_rate_ = 20.0;
  double startup_delay_ = 2.0;
  double odom_timeout_ = 1.0;
  double distance_tolerance_ = 0.10;
  double desired_speed_ = 0.5;
  double min_linear_speed_ = 0.05;
  double max_angular_speed_ = 0.8;
  double max_linear_accel_ = 0.4;
  double max_linear_decel_ = 0.6;
  double max_angular_accel_ = 1.0;
  double regulated_min_radius_ = 1.0;
  double max_lateral_accel_ = 0.5;
  double approach_distance_ = 2.0;
  double lookahead_distance_ = 0.6;
  double min_lookahead_distance_ = 0.4;
  double max_lookahead_distance_ = 1.2;
  double lookahead_time_ = 1.0;
  double curve_lookahead_gain_ = 2.0;
  double curvature_sample_distance_ = 0.4;
  double curve_speed_safety_factor_ = 0.85;
  double path_search_distance_ = 6.0;
  double path_backtrack_distance_ = 0.5;
  bool start_from_nearest_ = false;
  bool rotate_in_place_ = true;
  double rotate_in_place_threshold_ = 1.0;
  double rotate_in_place_speed_ = 0.4;
  double actual_path_min_distance_ = 0.05;
  int actual_path_max_poses_ = 5000;

  bool has_latest_odom_ = false;
  bool has_path_ = false;
  bool progress_initialized_ = false;
  bool finished_ = false;
  bool control_enabled_ = true;
  bool waiting_for_localization_ = false;
  bool has_scan_after_request_ = false;
  bool localization_stability_started_ = false;
  bool startup_delay_started_ = false;
  bool has_last_actual_position_ = false;

  ros::WallTime startup_wall_time_;
  ros::WallTime localization_request_wall_time_;
  ros::WallTime localization_stability_wall_time_;
  ros::Time initial_pose_sent_stamp_;
  double localization_anchor_x_ = 0.0;
  double localization_anchor_y_ = 0.0;
  double localization_anchor_yaw_ = 0.0;
  double path_progress_ = 0.0;
  double last_linear_command_ = 0.0;
  double last_angular_command_ = 0.0;
  geometry_msgs::Point last_actual_position_;

  nav_msgs::Odometry latest_odom_;
  nav_msgs::Path actual_path_;
  std::vector<geometry_msgs::Point> path_points_;
  std::vector<double> cumulative_lengths_;
  std::vector<double> path_speed_limits_;
};

}  // namespace diff_tracked_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_rpp_controller");
  diff_tracked_control::PathRppController controller;
  if (!controller.Initialize()) {
    ROS_FATAL("Failed to initialize path RPP controller");
    return 1;
  }
  ros::spin();
  return 0;
}
