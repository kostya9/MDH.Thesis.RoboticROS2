#include <cstdio>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <utility>
#include <cmath>
#include <float.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "pathfinder/msg/move.hpp"
#include "path.hh"

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace gazebo_msgs::msg;
using namespace pathfinder::msg;
using namespace nav_msgs::msg;

class PathFinder : public rclcpp::Node
{
public:
  PathFinder() : Node("pathfinder")
  {
    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    _laserSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/myrobot/laser/out", qos, std::bind(&PathFinder::OnLaserScan, this, std::placeholders::_1));
    _velocityPub = this->create_publisher<Twist>("/myrobot/cmd_vel", rclcpp::QoS(10));
    _odomSub = this->create_subscription<Odometry>("/myrobot/odom", rclcpp::QoS(10), std::bind(&PathFinder::OnOdom, this, std::placeholders::_1));
    _targetSub = this->create_subscription<Move>("/pathfinder/move", rclcpp::QoS(10), std::bind(&PathFinder::OnMoveCommand, this, std::placeholders::_1));
    _pointCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/myrobot/collisions", rclcpp::QoS(10));
    _state = FINISH;
    _path = Path();
    _hasPose = false;
  }
private:
  rclcpp::Subscription<LaserScan>::SharedPtr _laserSub;
  rclcpp::Subscription<Move>::SharedPtr _targetSub;
  rclcpp::Publisher<Twist>::SharedPtr _velocityPub;
  rclcpp::Subscription<Odometry>::SharedPtr _odomSub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointCloudPub;
  bool _hasPose;
  PoseWithCovariance _curPos;
  Move::SharedPtr _target;
  int _state;
  PoseWithCovariance _obstacleEnd;
  Path _path;

  void UpdateMap(const LaserScan::SharedPtr scanInfo)
  {
    auto pitch = GetCurrentPitch();
    if(std::abs(pitch) >= (M_PI / 1024)) 
    {
      return;
    }

    auto curAngle = scanInfo->angle_min;
    for(size_t i = 0; i < scanInfo->ranges.size(); i++) {
      auto d = scanInfo->ranges.at(i);
      auto yaw = curAngle;

      if(d > scanInfo->range_min && d < scanInfo->range_max) 
      {
        _path.AddCollision(d, yaw);
      }

      curAngle += scanInfo->angle_increment;
    }
  }

  double GetCurrentPitch() 
  {
    auto q = _curPos.pose.orientation;
    return std::asin(-2.0*(q.x*q.z - q.w*q.y));
  }
  

  void PublishMap()
  {
    auto collisions = _path.GetCollisions();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (auto &&c : collisions)
    {
      auto pt = pcl::PointXYZ();
      pt.x = c.first;
      pt.y = c.second;
      pt.z = 0.0;
      cloud.points.push_back(pt);
    }

    auto pc2_msg = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(cloud, pc2_msg);
    pc2_msg.header.frame_id = "odom_demo";
    pc2_msg.header.stamp = now();
    _pointCloudPub->publish(pc2_msg);
  }

  double getTargetYaw()
  {

    auto tX = _target->x;
    auto tY = _target->y;
    auto sY = _curPos.pose.position.y;
    auto sX = _curPos.pose.position.x;

    // Vector difference between positions of target and source
    auto deltaX = tX - sX;
    auto deltaY = tY - sY;

    auto cosYaw = (deltaX) / (std::sqrt(deltaX * deltaX + deltaY * deltaY));
    auto acosYaw = std::acos(cosYaw);

    if(tY >= sY) 
    {
      return acosYaw;
    }
    else 
    {
      return -acosYaw;
    }
  }

  double getCurrentYaw()
  {
    auto q = _curPos.pose.orientation;
    return std::atan2(2.0 * (q.z * q.w + q.x * q.y), - 1.0 + 2.0 * (q.w * q.w + q.x * q.x));
  }

  double getYawDiff(double yawLeft, double yawRight)
  {
    return std::fmod( yawLeft - yawRight + 3 * M_PI, 2 * M_PI) - M_PI;
  }

  double getDistanceTo(double x, double y)
  {
    auto xDist = std::pow(_curPos.pose.position.x - x, 2);
    auto yDist = std::pow(_curPos.pose.position.y - y, 2);
    return std::sqrt(xDist + yDist);
  }

  bool IsNextToCollision(const LaserScan::SharedPtr scanInfo, double minDist)
  {
    auto total = scanInfo->ranges.size();
    for(std::size_t i = 0; i < total; i++)
    {
      if(scanInfo->ranges.at(i) <= minDist)
      {
        return true;
      }
    }

    return false;
  }

  double GetTurnAngle(const sensor_msgs::msg::LaserScan::SharedPtr scanInfo)
  {
    // Find two rays that hit
    int idxF = -1;
    int idxS = -1;
    float dist = FLT_MAX;
    for (auto i = 1u; i < scanInfo->ranges.size(); ++i) {
      auto prev = scanInfo->ranges.at(i - 1);
      auto cur = scanInfo->ranges.at(i);
      float curDist = (cur + prev) / 2;
      if(prev > scanInfo->range_min && prev < scanInfo->range_max && prev < OBSTACLE_MOVE_MIN_DIST && 
        cur > scanInfo->range_min && cur < scanInfo->range_max && cur < OBSTACLE_MOVE_MIN_DIST && 
        curDist < dist)
        {
          idxF = i - 1;
          idxS = i;
          dist = curDist;
        }
    }

    if(idxF == -1)
    {
      return 0;
    }

    auto a = scanInfo->ranges.at(idxF);
    auto b = scanInfo->ranges.at(idxS);
    auto alpha = scanInfo->angle_increment;
    auto c = std::sqrt(a * a + b * b - 2 * a * b * std::cos(alpha));
    auto beta = std::asin(a * std::sin(alpha) / c);
    auto betaComp = M_PI - beta;
    auto gamma = M_PI - scanInfo->angle_min - idxS * scanInfo->angle_increment;
    auto angleOfRobot = 2 * M_PI - gamma - betaComp;

    return angleOfRobot;
  }

  std::string toStateString(int stateInt)
  {
    switch(stateInt)
    {
      case TARGET: return "TARGET";
      case FINISH: return "FINISH";
      case OBSTACLE_MOVE: return "OBSTACLE_MOVE";
      case OBSTACLE_TURN: return "OBSTACLE_TURN";
      case FINISH_OBSTACLE_MOVE: return "FINISH_OBSTACLE_MOVE";
    }

    return "UNKNOWN";
  }

  void SwitchState(int targetState)
  {
    auto stateString = toStateString(_state);
    auto targetStateString = toStateString(targetState);
    RCLCPP_INFO(this->get_logger(), "Switching from state {%s} to state {%s}", stateString.c_str(), targetStateString.c_str());
    _state = targetState;
  }

  Twist::UniquePtr DoNothing()
  {
    auto twist = std::make_unique<Twist>();
    return twist;
  }

  Twist::UniquePtr MakeMoveTurnCommand(float turnAngle)
  {
      auto epsilon = 0.01;
      auto vectorLiner = Vector3();
      auto vectorAngular = Vector3();
      vectorLiner.x = MOVEMENT_SPEED;
      if(std::abs(turnAngle) > epsilon)
      {
        auto angularSpeed = 0.01 * turnAngle;
        vectorAngular.z = angularSpeed;
        vectorLiner.x = MOVEMENT_SPEED; 
      }
      

      auto command = std::make_unique<Twist>();
      command->linear = vectorLiner;
      command->angular = vectorAngular;
      return command;
  }

  Twist::UniquePtr MakeMoveCommand(float speed)
  {
      auto vectorLiner = Vector3();
      vectorLiner.x = speed;

      auto command = std::make_unique<Twist>();
      command->linear = vectorLiner;
      return command;
  }

  Twist::UniquePtr MakeTurnCommand(float angle)
  {
      auto vectorAngular = Vector3();
      vectorAngular.z = angle * 0.02;

      auto command = std::make_unique<Twist>();
      command->angular = vectorAngular;
      return command;
  }

  Twist::UniquePtr ObstacleTurn(const LaserScan::SharedPtr scanInfo)
  {
    auto epsilon = 0.01;
    auto turn = GetTurnAngle(scanInfo);

    if(std::abs(turn) < epsilon)
    {
      SwitchState(OBSTACLE_MOVE);
      return nullptr;
    }

    turn = std::copysign(M_PI / 4, turn);
    return MakeTurnCommand(turn);
  }

  Twist::UniquePtr GoTarget(const LaserScan::SharedPtr scanInfo)
  {
    auto distance = getDistanceTo(_target->x, _target->y);
    if(distance <= FINISH_DIST)
    {
      SwitchState(FINISH);
      return DoNothing();
    }

    if(IsNextToCollision(scanInfo, OBSTACLE_MIN_DIST))
    {
      SwitchState(OBSTACLE_TURN);
      return nullptr;
    }

    auto goalYaw = getYawDiff(getTargetYaw(), getCurrentYaw());
    if(distance < OBSTACLE_MOVE_MIN_DIST)
    {
      double epsilon = 0.01;
      if(std::abs(goalYaw) < epsilon)
      {
        return MakeMoveCommand(MOVEMENT_SPEED);
      }
      
      auto turn = std::copysign(M_PI, goalYaw);
      return MakeMoveTurnCommand(turn);
    }

    if(distance > OBSTACLE_MOVE_MIN_DIST && IsNextToCollision(scanInfo, OBSTACLE_MOVE_MIN_DIST))
    {
      auto turn = GetTurnAngle(scanInfo);
      return MakeMoveTurnCommand(turn);
    }

    
    return MakeMoveTurnCommand(goalYaw);
  }

  Twist::UniquePtr ObstacleMove(const LaserScan::SharedPtr scanInfo)
  {
    if(IsNextToCollision(scanInfo, OBSTACLE_MIN_DIST))
    {
      SwitchState(OBSTACLE_TURN);
      return nullptr;
    }

    for (auto i = 0u; i < scanInfo->ranges.size(); ++i) 
    {
      auto cur = scanInfo->ranges.at(i);
      if(cur < OBSTACLE_MOVE_MIN_DIST)
      {
        break;
      }

      _obstacleEnd = _curPos;
      SwitchState(FINISH_OBSTACLE_MOVE);
      return nullptr;
    }

    auto turn = GetTurnAngle(scanInfo);
    return MakeMoveTurnCommand(turn);
  }

  Twist::UniquePtr ObstacleEnd()
  {
    auto distance = getDistanceTo(_obstacleEnd.pose.position.x, _obstacleEnd.pose.position.y);

    if(distance >= 1)
    {
      SwitchState(TARGET);
      return nullptr;
    }

    return MakeMoveCommand(MOVEMENT_SPEED);
  }

  Twist::UniquePtr GetMovementFromState(const LaserScan::SharedPtr scanInfo)
  {
    switch (_state)
    {
      case TARGET: 
      {
        return GoTarget(scanInfo);
      } break;
      case OBSTACLE_TURN: 
      {
        return ObstacleTurn(scanInfo);
      } break;
      case OBSTACLE_MOVE: 
      {
        return ObstacleMove(scanInfo);
      } break;
      case FINISH_OBSTACLE_MOVE:
      {
        return ObstacleEnd();
      } break;
      case FINISH: 
      {
        return DoNothing();
      } break;
    }

    return DoNothing();
  }

  void OnLaserScan(const LaserScan::SharedPtr scanInfo)
  {
    if(!_hasPose) 
    {
      return;
    }

    this->UpdateMap(scanInfo);
    this->PublishMap();
    auto command = GetMovementFromState(scanInfo);

    if(command != nullptr)
    {
      _velocityPub->publish(*command);
    }
  }

  void OnOdom(Odometry::SharedPtr response) 
  {
    this->_curPos = response->pose;
    _path.SetPosition(_curPos.pose.position.x, _curPos.pose.position.y, getCurrentYaw());
    _hasPose = true;
  }

  void OnMoveCommand(const Move::SharedPtr moveInfo)
  {
    this->_target = moveInfo;
    RCLCPP_INFO(this->get_logger(), "Received Move: %.3f, %.3f", _target->x, _target->y);
    SwitchState(TARGET);
  }

  static constexpr int TARGET = 0;
  static constexpr int OBSTACLE_MOVE = 2;
  static constexpr int OBSTACLE_TURN = 3;
  static constexpr int FINISH_OBSTACLE_MOVE = 1;
  static constexpr int FINISH = 4;
  static constexpr float OBSTACLE_MIN_DIST = 0.5;
  static constexpr float FINISH_DIST = 0.8;
  static constexpr float OBSTACLE_MOVE_MIN_DIST = 1.5;
  static constexpr float MOVEMENT_SPEED = 0.06;
};

// RCLCPP_COMPONENTS_REGISTER_NODE(PathFinder)


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PathFinder>();

  while(rclcpp::ok())
  {
    rclcpp::spin(node);
  }

  rclcpp::shutdown();

  return 0;
}
