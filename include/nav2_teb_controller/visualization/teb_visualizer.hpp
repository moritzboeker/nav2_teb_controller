#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include "nav2_teb_controller/core/timed_elastic_band.hpp"
#include "nav2_teb_controller/core/teb_utils.hpp"
#include "nav2_teb_controller/core/footprint.hpp"

namespace nav2_teb_controller {

class TEBVisualizer
{
public:
  explicit TEBVisualizer(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : node_(node){}

  /// @brief Publishes the local plan (TEB as nav_msgs::Path)
  void publishLocalPlan(const TimedElasticBand& teb, const std::string& frame_id)
  {
    if (local_plan_pub_->get_subscription_count() == 0) return;

    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = frame_id;

    for (std::size_t i = 0; i < teb.sizePoses(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = teb.pose(i).x();
      ps.pose.position.y = teb.pose(i).y();
      tf2::Quaternion q;
      q.setRPY(0, 0, teb.pose(i).theta());
      ps.pose.orientation = tf2::toMsg(q);
      path.poses.push_back(ps);
    }
    local_plan_pub_->publish(path);
  }

  /// @brief Publishes the current lookahead window of the global plan
  void publishLookaheadPlan(const nav_msgs::msg::Path& transformed_plan)
  {
    if (lookahead_plan_pub_->get_subscription_count() > 0)
      lookahead_plan_pub_->publish(transformed_plan);
    if (lookahead_goal_pub_->get_subscription_count() > 0)
      lookahead_goal_pub_->publish(transformed_plan.poses.back());
  }

  /// @brief Publishes TEB poses as PoseArray
  void publishTEBPoses(const TimedElasticBand& teb, const std::string& frame_id)
  {
    if (teb_poses_pub_->get_subscription_count() == 0) return;

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = node_->now();
    pose_array.header.frame_id = frame_id;

    for (std::size_t i = 0; i < teb.sizePoses(); ++i) {
      geometry_msgs::msg::Pose p;
      p.position.x = teb.pose(i).x();
      p.position.y = teb.pose(i).y();
      tf2::Quaternion q;
      q.setRPY(0, 0, teb.pose(i).theta());
      p.orientation = tf2::toMsg(q);
      pose_array.poses.push_back(p);
    }
    teb_poses_pub_->publish(pose_array);
  }

  /// @brief Publishes obstacles as MarkerArray from costmap_converter ObstacleArrayMsg
  void publishObstacles(
    costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obstacles,
    const std::string& frame_id)
  {
    if (!obstacles) return;
    if (obstacles_pub_->get_subscription_count() == 0) return;
    if (obstacles->obstacles.empty()) return;

    visualization_msgs::msg::MarkerArray marker_array;
    const auto stamp = node_->now();

    int id = 0;
    for (const auto& obs : obstacles->obstacles)
    {
      if (obs.polygon.points.empty()) continue;

      visualization_msgs::msg::Marker m;
      m.header.stamp    = stamp;
      m.header.frame_id = frame_id;
      m.ns              = "teb_obstacles";
      m.id              = id++;
      m.action          = visualization_msgs::msg::Marker::ADD;
      m.lifetime        = rclcpp::Duration::from_seconds(0.5);
      m.pose.position.z = 0.1;
      m.pose.orientation.w = 1.0;

      if (obs.polygon.points.size() == 1)
      {
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.pose.position.x = (double)obs.polygon.points[0].x;
        m.pose.position.y = (double)obs.polygon.points[0].y;
        m.scale.x = 0.1; m.scale.y = 0.1; m.scale.z = 0.2;
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.8;
      }
      else
      {
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.scale.x = 0.05;
        m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 0.8;
        for (const auto& pt : obs.polygon.points) {
          geometry_msgs::msg::Point p;
          p.x = (double)pt.x; p.y = (double)pt.y; p.z = 0.1;
          m.points.push_back(p);
        }
        if (!m.points.empty())
          m.points.push_back(m.points.front());
      }
      marker_array.markers.push_back(m);
    }
    obstacles_pub_->publish(marker_array);
  }

  void publishCurvatureRadii(const TimedElasticBand& teb, const std::string& frame_id)
  {
    if (radius_markers_pub_->get_subscription_count() == 0) return;

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.clear();  // Clear alte Marker
    
    // Parameter für saubere Darstellung
    constexpr double kappa_min = 0.1;     // R > 20m → keine Anzeige
    constexpr double radius_max_viz = 5.0; // Max Pfeillänge 5m
    constexpr double R_min_robot = 0.0;    // Engste Kurve Robot
    constexpr double arrow_scale = 0.2;   // Pfeil = 12% Radius
    
    for (size_t i = 1; i < teb.sizePoses() - 1; ++i)
    {
      const PoseSE2& p_before = teb.pose(i-1);
      const PoseSE2& pose_mid = teb.pose(i);
      const PoseSE2& p_after = teb.pose(i+1);
      
      // 1. κ berechnen
      double kappa = computeCurvature(p_before, pose_mid, p_after);
      
      // 2. Filter: Gerade Segmente ausblenden
      if (fabs(kappa) < kappa_min)
        continue;
      
      // 3. Radius limitieren
      double radius = std::clamp(1.0 / fabs(kappa), R_min_robot, radius_max_viz);
      
      // 4. Radius-Markierung (Kamm-Pfeil)
      
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = node_->now();
      marker.ns = "teb_radii";
      marker.id = static_cast<int>(i);
      // marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(0.1);  // Kurzlebig
      
      // Position am Mittelpunkt, leicht erhöht
      marker.pose.position.x = pose_mid.x();
      marker.pose.position.y = pose_mid.y();
      marker.pose.position.z = radius * arrow_scale;  // Mitte des Zylinders
      // marker.pose.position.z = 0.15; 
      marker.pose.orientation.w = 1.0;

      // Marker Orientierung
      // double mid_heading = pose_mid.theta();
      // double arrow_yaw = mid_heading + M_PI / 2.0;  // 90° nach rechts
      // if (kappa > 0) arrow_yaw -= M_PI;  // Links kurven → Pfeil nach innen
      // tf2::Quaternion quat;
      // quat.setRPY(0, 0, arrow_yaw);
      // marker.pose.orientation = tf2::toMsg(quat);
      // Orientierung irrelevant für Zylinder
      marker.pose.orientation.w = 1.0;
      
      // Pfeil-Größe proportional zum Radius
      // marker.scale.x = radius * arrow_scale;  // Länge
      // marker.scale.y = 0.04;                  // Breite
      // marker.scale.z = 0.04;                  // Höhe
      // Höhe = 2 * Radius (von -R bis +R)
      marker.scale.z = 2.0 * radius * arrow_scale;  // 10% Skalierung
      marker.scale.x = 0.08;  // Dicke
      marker.scale.y = 0.08;
      
      // Farbe: Rot=eng (gefährlich), Grün=weit (sicher)
      double hue = (radius - R_min_robot) / (radius_max_viz - R_min_robot);
      marker.color.r = 1.0 - hue;
      marker.color.g = hue;
      marker.color.b = 0.0;
      marker.color.a = 0.85;
      
      marker_array.markers.push_back(marker);
    }
    radius_markers_pub_->publish(marker_array);
  }

  void publishFootprint(const PoseSE2& current_pose, const Footprint& footprint, const std::string& frame_id)
  {
    if (footprint_markers_pub_->get_subscription_count() == 0) return;

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.clear();  // Clear alte Marker

    const double px = current_pose.x();
    const double py = current_pose.y();
    const double ct = std::cos(current_pose.theta());
    const double st = std::sin(current_pose.theta());

    const auto& circles = footprint.circles();
    for (int i = 0; i < static_cast<int>(circles.size()); ++i) 
    {
      const auto& c   = circles[i];
      const double wx = px + ct * c.offset.x() - st * c.offset.y();
      const double wy = py + st * c.offset.x() + ct * c.offset.y();

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = node_->now();
      marker.ns = "teb_footprint";
      marker.id = static_cast<int>(i);
      // marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(0.1);  // Kurzlebig
      marker.pose.position.x = wx;
      marker.pose.position.y = wy;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 2.0 * c.radius;
      marker.scale.y = 2.0 * c.radius;
      marker.scale.z = 0.05;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.4f;
      marker_array.markers.push_back(marker);
    }
    footprint_markers_pub_->publish(marker_array);
  }

  nav2_util::CallbackReturn on_configure()
  {
    local_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
    lookahead_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("teb_lookahead_plan", 1);
    lookahead_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("teb_lookahead_goal", 1);
    teb_poses_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("teb_poses", 1);
    obstacles_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("teb_obstacles", 1);
    radius_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("teb_radius_markers", 1);
    footprint_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("footprint_markers", 1);
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate()
  {
    local_plan_pub_->on_activate();
    lookahead_plan_pub_->on_activate();
    lookahead_goal_pub_->on_activate();
    teb_poses_pub_->on_activate();
    obstacles_pub_->on_activate();
    radius_markers_pub_->on_activate();
    footprint_markers_pub_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr lookahead_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_goal_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr teb_poses_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr radius_markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_markers_pub_;
};

}  // namespace nav2_teb_controller
