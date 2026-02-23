#pragma once
#include <Eigen/Dense>

class DiffDriveController {
 public:
  DiffDriveController()
      : _max_v(1.1f), 
        _max_w(2.0f), 
        _tolerance(0.2f), 
        _k_rho(1.4f), 
        _k_w(1.0f), 
        _done(true),
        _obstacle_avoidance(false) {}
  void update(const Eigen::Isometry2f& current_pose);

  inline void setWaypoints(const std::vector<Eigen::Isometry2f>& waypoints) {
    _waypoints = waypoints;
    _current_waypoint = 0;
    _done = false;
  }

  inline void setLaserMeasurements(
      const std::vector<Eigen::Vector2f> measurements) {
    _measurements = measurements;
  }

  inline void setObstacleAvoidance() { _obstacle_avoidance = true; }

  inline const float& getV() const { return _output_v; }
  inline const float& getW() const { return _output_w; }
  inline bool isDone() const { return _done; }

 protected:
  std::vector<Eigen::Isometry2f> _waypoints;
  unsigned int _current_waypoint = 0;
  std::vector<Eigen::Vector2f> _measurements;
  float _max_v, _max_w;
  float _tolerance;
  float _k_rho, _k_w;
  bool _done;
  bool _obstacle_avoidance;

  float _output_v, _output_w;
};