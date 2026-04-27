#pragma once

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <nav2_teb_controller/obstacles/esdf.hpp>

namespace nav2_teb_controller
{
class PlannerBase {
public:
  virtual ~PlannerBase() = default;
  virtual bool plan(const nav_msgs::msg::Path&,
                    const geometry_msgs::msg::Twist&) = 0;
  virtual bool hasDiverged() = 0;               
  virtual void clear() = 0;
};

template<typename TEBType>
class PlannerInterface : public PlannerBase {
public:
  virtual const TEBType& getTEB() const = 0;
  virtual void setFeedback(
    const ackermann_msgs::msg::AckermannDrive& feedback) = 0;
  virtual void updateObstacleContainer (
     costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obstacle_array = nullptr) = 0;
  virtual void setObstacleMap(const ObstacleMap2D* esdf) = 0;
};
}  // namespace nav2_teb_controller