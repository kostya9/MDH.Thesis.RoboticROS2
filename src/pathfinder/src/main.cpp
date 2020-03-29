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
    _path = Path(0.5);
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

  double getTargetYaw(int x, int y)
  {
    auto sY = _curPos.pose.position.y;
    auto sX = _curPos.pose.position.x;

    // Vector difference between positions of target and source
    auto deltaX = x - sX;
    auto deltaY = y - sY;

    auto cosYaw = (deltaX) / (std::sqrt(deltaX * deltaX + deltaY * deltaY));
    auto acosYaw = std::acos(cosYaw);

    if(y >= sY) 
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

  void OnLaserScan(const LaserScan::SharedPtr scanInfo)
  {
    if(!_hasPose) 
    {
      return;
    }

    this->UpdateMap(scanInfo);
    this->PublishMap();
    auto path = this->_path.GetPathToTarget();
    auto firstPoint = path->at(0);
    auto angle = getYawDiff(getCurrentYaw(), getTargetYaw(firstPoint.first, firstPoint.second));
    auto command = MakeMoveTurnCommand(angle);

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
    this->_path.SetTarget(moveInfo->x, moveInfo->y);
    RCLCPP_INFO(this->get_logger(), "Received Move: %.3f, %.3f", _target->x, _target->y);
  }
  
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
