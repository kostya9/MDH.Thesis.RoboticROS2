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
#include "pathfinder/msg/rotate.hpp"
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
    _rotateSub = this->create_subscription<Rotate>("/pathfinder/rotate", rclcpp::QoS(10), std::bind(&PathFinder::OnRotateCommand, this, std::placeholders::_1));
    _pointCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/myrobot/collisions", rclcpp::QoS(10));
    _pathPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/myrobot/path", rclcpp::QoS(10));
    _path = std::make_unique<Path>(this->get_logger(), 0.25);
    _hasPose = false;
  }
private:
  rclcpp::Subscription<LaserScan>::SharedPtr _laserSub;
  rclcpp::Subscription<Move>::SharedPtr _targetSub;
  rclcpp::Subscription<Rotate>::SharedPtr _rotateSub;
  rclcpp::Publisher<Twist>::SharedPtr _velocityPub;
  rclcpp::Subscription<Odometry>::SharedPtr _odomSub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointCloudPub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pathPub;
  bool _hasPose;
  PoseWithCovariance _curPos;
  Move::SharedPtr _target;
  PoseWithCovariance _obstacleEnd;
  std::unique_ptr<Path> _path;
  int shakes = 0;

  bool rotate = false;
  double rotateSpeed = 0;

  bool UpdateMap(const LaserScan::SharedPtr scanInfo)
  {
    auto pitch = GetCurrentPitch();
    if(std::abs(pitch) >= (M_PI / 1024)) 
    {
      return false;
    }

    auto curAngle = scanInfo->angle_min;
    bool addedCollision = false;
    for(size_t i = 0; i < scanInfo->ranges.size(); i++) {
      auto d = scanInfo->ranges.at(i);
      auto yaw = curAngle;

      if(d > scanInfo->range_min && d < scanInfo->range_max) 
      {
        addedCollision = _path->AddCollision(d, yaw) || addedCollision; 
      }

      curAngle += scanInfo->angle_increment;
    }

    return addedCollision;
  }

  double GetCurrentPitch() 
  {
    auto q = _curPos.pose.orientation;
    return std::asin(-2.0*(q.x*q.z - q.w*q.y));
  }

  void PublishCoordinates(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
    std::vector<Coordinate> & coordinates, uint8_t g = 0) 
    {
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      for (auto &&c : coordinates)
      {
        auto pt = pcl::PointXYZRGB();
        pt.x = c.first;
        pt.y = c.second;
        pt.z = 0.0;
        pt.g = g;
        cloud.points.push_back(pt);
      }

      auto pc2_msg = sensor_msgs::msg::PointCloud2();
      pcl::toROSMsg(cloud, pc2_msg);
      pc2_msg.header.frame_id = "odom_demo";
      pc2_msg.header.stamp = now();
      publisher->publish(pc2_msg);
    }
  

  void PublishMap()
  {
    auto collisions = _path->GetCollisions();
    PublishCoordinates(_pointCloudPub, collisions);
  }

  double getTargetYaw(double x, double y)
  {
    auto sY = _curPos.pose.position.y;
    auto sX = _curPos.pose.position.x;

    // Vector difference between positions of target and source
    auto deltaX = x - sX;
    auto deltaY = y - sY;

    auto atan = std::atan2(deltaY, deltaX);
    return atan;
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
      auto vectorLinear = Vector3();
      auto vectorAngular = Vector3();
      vectorLinear.x = MOVEMENT_SPEED;
      if(std::abs(turnAngle) > epsilon)
      {
        vectorAngular.z = turnAngle * angular_k_;
        vectorLinear.x = MOVEMENT_SPEED;
      }

      auto command = std::make_unique<Twist>();
      command->linear = vectorLinear;
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

  Twist::UniquePtr GetLaserScanCommand(const LaserScan::SharedPtr scanInfo)
  {
    if(!_hasPose) 
    {
      return nullptr;
    }

    if(this->UpdateMap(scanInfo))
    {
      this->PublishMap();
    }

    if(rotate)
    {
      static int i = 9;
      auto path = this->_path->GetPathToTarget();
      if(path->size() > 0 && i++ % 10 == 0)
      {
        i = 0;
        auto firstPoint = path->at(0);
        auto angle = -getYawDiff(getCurrentYaw(), getTargetYaw(firstPoint.first, firstPoint.second));
        RCLCPP_INFO(this->get_logger(), "My pos: %.3f, %.3f; Next point: %.3f, %.3f; angle between: %.3f", _curPos.pose.position.x, _curPos.pose.position.y, firstPoint.first, firstPoint.second, angle);
      }

      return MakeTurnCommand(rotateSpeed);
    }

    if(_target == nullptr)
    {
      return nullptr;
    }

    auto path = this->_path->GetPathToTarget();

    Twist::UniquePtr command = nullptr;
    auto pitch = GetCurrentPitch();
    if(std::abs(pitch) >= (M_PI / 128)) // shaky robot 
    {
      if(shakes < 1024) {
        shakes++;
      }
    } 
    else if(shakes > 0)
    {
      shakes--;
    } 

    if(shakes > 512) // stabilizing shakiness 
    {
      return MakeMoveCommand(0);
    }
    else if(path->size() > 0) // normal behavior
    {
      PublishCoordinates(_pathPub, *path, 255);
      auto firstPoint = path->at(0);

      auto curYaw = getCurrentYaw();
      auto tarYaw = getTargetYaw(firstPoint.first, firstPoint.second);
      auto angle = -getYawDiff(curYaw, tarYaw);

      if(fabs(angle) > M_PI / 4) {
        auto turnSpeed = copysign(M_PI / 4, angle);
        RCLCPP_INFO(this->get_logger(), "My pos: %.3f, %.3f; Next point: %.3f, %.3f; angle between: %.3f (cur: %.3f,tar: %.3f), using turn speed %.3f", 
          _curPos.pose.position.x, _curPos.pose.position.y, firstPoint.first, firstPoint.second, angle, curYaw, tarYaw, turnSpeed);
        return MakeTurnCommand(turnSpeed);
      } else {
        return MakeMoveTurnCommand(angle);
      }
    }
    else // nowhere to go 
    {
      return  MakeMoveCommand(0);
    }

    
  }

  void OnLaserScan(const LaserScan::SharedPtr scanInfo)
  {
    auto nullcmd = MakeMoveCommand(0);
    try
    {
      auto command = GetLaserScanCommand(scanInfo);
      if(command != nullptr)
      {
        _velocityPub->publish(*command);
      }
    }
    catch (std::exception const &exc)
    {
        std::cerr << "Exception caught " << exc.what() << "\n";
        _velocityPub->publish(*nullcmd);
    }
    catch (...)
    {
        std::cerr << "Unknown exception caught\n";
        _velocityPub->publish(*nullcmd);
    }
  }

  void OnOdom(Odometry::SharedPtr response) 
  {
    this->_curPos = response->pose;
    _path->SetPosition(_curPos.pose.position.x, _curPos.pose.position.y, getCurrentYaw());
    _hasPose = true;
  }

  void OnMoveCommand(const Move::SharedPtr moveInfo)
  {
    this->rotate = false;
    this->_target = moveInfo;
    this->_path->SetTarget(moveInfo->x, moveInfo->y);
    RCLCPP_INFO(this->get_logger(), "Received Move: %.3f, %.3f", _target->x, _target->y);
  }

  void OnRotateCommand(const Rotate::SharedPtr rotInfo)
  {
    this->rotate = true;
    this->rotateSpeed = rotInfo->speed;
    RCLCPP_INFO(this->get_logger(), "Received Rotate: %.3f", rotateSpeed);
  }
  
  static constexpr float MOVEMENT_SPEED = 0.08;

  /// \brief Scale angular velocity, chosen by trial and error
  double angular_k_ = 0.06;

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
