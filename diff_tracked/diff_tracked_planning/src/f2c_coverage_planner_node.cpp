#include <cmath>
#include <exception>
#include <string>

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "fields2cover.h"

namespace diff_tracked_planning {

class CoveragePlannerNode {
 public:
  CoveragePlannerNode() : private_nh_("~") {
    LoadParameters();
    path_publisher_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
  }

  bool PlanAndPublish() {
    if (!ValidateParameters()) {
      return false;
    }

    try {
      F2CLinearRing boundary{
          F2CPoint(origin_x_, origin_y_),
          F2CPoint(origin_x_ + field_length_, origin_y_),
          F2CPoint(origin_x_ + field_length_, origin_y_ + field_width_),
          F2CPoint(origin_x_, origin_y_ + field_width_),
          F2CPoint(origin_x_, origin_y_)};

      F2CCell field;
      field.addRing(boundary);

      F2CRobot robot(robot_width_, coverage_width_);
      robot.setMinTurningRadius(min_turning_radius_);
      robot.setCruiseVel(cruise_speed_);
      robot.setTurnVel(turn_speed_);

      f2c::Options options;
      options.hg_swaths = headland_swaths_;

      F2CPath coverage_path = f2c::planCovPath(robot, field, options);
      coverage_path.discretize(path_resolution_);

      const nav_msgs::Path message = ToRosPath(coverage_path);
      if (message.poses.empty()) {
        ROS_ERROR("Fields2Cover returned an empty path");
        return false;
      }

      path_publisher_.publish(message);
      ROS_INFO("Published coverage path: %zu poses, %.2f m, topic %s",
               message.poses.size(), coverage_path.length(),
               path_topic_.c_str());
      return true;
    } catch (const std::exception& exception) {
      ROS_ERROR("Fields2Cover planning failed: %s", exception.what());
      return false;
    }
  }

 private:
  void LoadParameters() {
    private_nh_.param("frame_id", frame_id_, std::string("map"));
    private_nh_.param("path_topic", path_topic_,
                      std::string("/coverage_path"));

    private_nh_.param("origin_x", origin_x_, 0.0);
    private_nh_.param("origin_y", origin_y_, 0.0);
    private_nh_.param("field_length", field_length_, 30.0);
    private_nh_.param("field_width", field_width_, 20.0);

    private_nh_.param("robot_width", robot_width_, 1.0);
    private_nh_.param("coverage_width", coverage_width_, 2.0);
    private_nh_.param("min_turning_radius", min_turning_radius_, 1.0);
    private_nh_.param("cruise_speed", cruise_speed_, 0.5);
    private_nh_.param("turn_speed", turn_speed_, 0.25);

    private_nh_.param("headland_swaths", headland_swaths_, 3);
    private_nh_.param("path_resolution", path_resolution_, 0.2);
  }

  bool ValidateParameters() const {
    if (frame_id_.empty() || path_topic_.empty()) {
      ROS_ERROR("~frame_id and ~path_topic must not be empty");
      return false;
    }
    if (field_length_ <= 0.0 || field_width_ <= 0.0) {
      ROS_ERROR("~field_length and ~field_width must be positive");
      return false;
    }
    if (robot_width_ <= 0.0 || coverage_width_ <= 0.0) {
      ROS_ERROR("~robot_width and ~coverage_width must be positive");
      return false;
    }
    if (min_turning_radius_ <= 0.0 || path_resolution_ <= 0.0) {
      ROS_ERROR("~min_turning_radius and ~path_resolution must be positive");
      return false;
    }
    if (cruise_speed_ <= 0.0 || turn_speed_ <= 0.0) {
      ROS_ERROR("~cruise_speed and ~turn_speed must be positive");
      return false;
    }
    if (headland_swaths_ < 1) {
      ROS_ERROR("~headland_swaths must be at least 1");
      return false;
    }
    return true;
  }

  nav_msgs::Path ToRosPath(const F2CPath& path) const {
    nav_msgs::Path message;
    message.header.frame_id = frame_id_;
    message.header.stamp = ros::Time::now();
    message.poses.reserve(path.size() + 1);

    for (const auto& state : path) {
      message.poses.push_back(ToPose(state.point, state.angle, message.header));
    }

    if (!path.getStates().empty()) {
      const auto& last_state = path.back();
      const F2CPoint last_point = last_state.atEnd();
      message.poses.push_back(
          ToPose(last_point, last_state.angle, message.header));
    }
    return message;
  }

  geometry_msgs::PoseStamped ToPose(
      const F2CPoint& point, double yaw,
      const std_msgs::Header& header) const {
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = point.getX();
    pose.pose.position.y = point.getY();
    pose.pose.position.z = point.getZ();
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    return pose;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher path_publisher_;

  std::string frame_id_;
  std::string path_topic_;
  double origin_x_ {0.0};
  double origin_y_ {0.0};
  double field_length_ {30.0};
  double field_width_ {20.0};
  double robot_width_ {1.0};
  double coverage_width_ {2.0};
  double min_turning_radius_ {1.0};
  double cruise_speed_ {0.5};
  double turn_speed_ {0.25};
  int headland_swaths_ {3};
  double path_resolution_ {0.2};
};

}  // namespace diff_tracked_planning

int main(int argc, char** argv) {
  ros::init(argc, argv, "f2c_coverage_planner");
  diff_tracked_planning::CoveragePlannerNode node;
  if (!node.PlanAndPublish()) {
    return 1;
  }
  ros::spin();
  return 0;
}
