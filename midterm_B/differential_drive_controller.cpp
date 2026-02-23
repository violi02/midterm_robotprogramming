#include "rp_controller/differential_drive_controller.h"

#include <iostream>

void DiffDriveController::update(const Eigen::Isometry2f& current_pose) {
  if (_done) {
    return;
  }

  // current pose world->robot
  // target_pos_in_world target->world

  std::cout << "Current pose: " << current_pose.translation().x() << " "
            << current_pose.translation().y() << std::endl;
  
  const auto& target_pose_in_world =  _waypoints[_current_waypoint];
  
  const auto& target_pose_in_robot = current_pose.inverse() * target_pose_in_world;

  Eigen::Vector2f delta_pos = Eigen::Vector2f::Zero();
  
   delta_pos = target_pose_in_robot.translation();
   
   const auto delta_yaw = std::atan2(delta_pos.y(), delta_pos.x());

  // HINT: The greater the distances to the target, the greater the velocities (use the gains)

   _output_v = delta_pos.norm()*_k_rho;
   if (_output_v > _max_v) _output_v = _max_v;
   if (_output_v < -_max_v) _output_v = -_max_v;
  
   _output_w = delta_yaw*_k_w;
   if (_output_w > _max_w) _output_w = _max_w;
   if (_output_w < -_max_w) _output_w = -_max_w;


  //***************  Begin of obstacle avoidance procedure ***************//
  if (_obstacle_avoidance) {
    // HINTS:
    // - Use the laser measurements to avoid obstacles
    // - Activate the obstacle avoidance when the robot is close to an obstacle
    // - Check if there are more points on the left or on the right (the azimuth of a point in the laser scan frame is atan2(y, x))
    // - If there are more points on the right, turn left and vice versa
    // - The rotational velocity should be proportional to the angle to the closest obstacle (0 when the obstacle is at +/-3.14 radians)

    // Note: The above HINTS are taken from our implementation, you can come up with your own solution
  }

  //***************  End of obstacle avoidance procedure ***************//

  // Check if the robot is close to the target waypoint
  if (delta_pos.norm() < _tolerance) {
    // Move to the next waypoint
    _current_waypoint++;
    std::cerr << "New waypoint" << std::endl;
    if (_current_waypoint >= _waypoints.size()) {
      std::cerr << "Completed path" << std::endl;
      _done = true;
      _output_v = 0.0f;
      _output_w = 0.0f;
    }
  }

  return;
}
