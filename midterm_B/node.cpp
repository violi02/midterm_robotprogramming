#include "rp_controller/node.h"

#include <Eigen/Dense>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rp_controller/differential_drive_controller.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <mutex>

DifferentialDriveControllerNode::DifferentialDriveControllerNode()
    : Node("differential_drive_controller_node") {
  // Declare the parameters
  this->declare_parameter<std::string>("laser_ns");
  this->declare_parameter<std::string>("base_link_ns");

  // Check if the parameters are set
  if (!this->has_parameter("laser_ns") ||
      !this->has_parameter("base_link_ns")) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Parameters 'laser_ns' and 'base_link_ns' are required but not set.");
    rclcpp::shutdown();
    throw std::runtime_error(
        "Parameters 'laser_ns' and 'base_link_ns' are required but not set.");
  }

  // Get the parameter values
  this->get_parameter("laser_ns", _laser_ns);
  this->get_parameter("base_link_ns", _base_link_ns);

  _controller = std::make_shared<DiffDriveController>();
  _timer = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DifferentialDriveControllerNode::update, this));

  _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);


  
  //convention on Ros topics
  _twist_pub = this->create_publisher<geometry_msgs::msg::Twist>(
    _base_link_ns + std::string("/cmd_vel"), 10);

  _scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    _laser_ns + std::string("/scan"), 10,std::bind(&DifferentialDriveControllerNode::laserCallback, this,
    std::placeholders::_1));

  _path_sub = this->create_subscription<nav_msgs::msg::Path>(_base_link_ns + std::string("/path"), 10,
    std::bind(&DifferentialDriveControllerNode::pathCallback, this,std::placeholders::_1));

}

void DifferentialDriveControllerNode::pathCallback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  // Convert the nav_msgs::Path to a vector of Eigen::Isometry2f
  std::vector<Eigen::Isometry2f> waypoints;
  for (const auto& pose : msg->poses) {
    std::cout << pose.pose.position.x << " " << pose.pose.position.y <<  std::endl;
  
    float _x_pos = pose.pose.position.x;
    float _y_pos = pose.pose.position.y;  
    auto q = pose.pose.orientation;
    Eigen::Isometry2f waypoint = Eigen::Isometry2f::Identity();
 
    waypoint.translation() = Eigen::Vector2f(_x_pos, _y_pos);
    // linear for rotation
 
    double s = 2.0 * (q.w * q.z + q.x * q.y);
    double c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(s, c);

    waypoint.linear() = Eigen::Rotation2Df(static_cast<float>(yaw)).toRotationMatrix();

    waypoints.push_back(waypoint);
  }
  RCLCPP_INFO(this->get_logger(), "Received %lu waypoints", waypoints.size());
 
  _controller->setWaypoints(waypoints);
}

void DifferentialDriveControllerNode::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(_measurements_mutex);

  LaserScan current_scan;
  current_scan.fromROSMessage(*msg);
  std::vector<Eigen::Vector2f> measurements = current_scan.toCartesian();
  
  _controller->setLaserMeasurements(measurements);
}


void DifferentialDriveControllerNode::update() {
  std::lock_guard<std::mutex> lock(_measurements_mutex);
  
  if (_controller->isDone()) {
    return;
  }
  geometry_msgs::msg::TransformStamped t;
  try {
    t = _tf_buffer->lookupTransform("map", "robot_1", tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_INFO(this->get_logger(),
                "Could not get transform from map to robot_1: %s", ex.what());
    return;
  }
  // Convert TransformStamped to a Eigen::Isometry2f

  float _x_pos = t.transform.translation.x;
  float _y_pos = t.transform.translation.y;
  auto q = t.transform.rotation;
  
  Eigen::Isometry2f current_pose = Eigen::Isometry2f::Identity();

  double s = 2.0 * (q.w * q.z + q.x * q.y);
  double c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(s, c);
  current_pose.linear() = Eigen::Rotation2Df(static_cast<float>(yaw)).toRotationMatrix();
  current_pose.translation() = Eigen::Vector2f(_x_pos, _y_pos);
  _controller->update(current_pose);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = _controller->getV();
  twist.angular.z = _controller->getW();

 
  _twist_pub->publish(twist);
}