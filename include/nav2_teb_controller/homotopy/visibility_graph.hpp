#pragma once
#include <vector>
#include <Eigen/Core>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>

namespace nav2_teb_controller {

/** @brief Single node in Visibility Graph */
struct VisibilityNode
{
  Eigen::Vector2d pos;
  int id = -1;
  bool is_start = false;
  bool is_goal = false;
};

/** @brief Edge between two visible nodes */
struct VisibilityEdge
{
  int from_id;
  int to_id;
  double cost;   // Euclidean distance
};

/**
 * @brief Visibility Graph data structure
 *
 * Nodes: start, goal, obstacle keypoints (inflated corners/tangents)
 * Edges: direct line-of-sight between nodes (no obstacle intersection)
 */
class VisibilityGraph
{
public:
  using ObstacleArray = costmap_converter_msgs::msg::ObstacleArrayMsg;

  /**
   * @brief Build visibility graph from obstacles + start + goal
   * @param start Start position
   * @param goal Goal position
   * @param obstacles Current obstacle array
   * @param inflation Obstacle inflation radius
   */
  void build(const Eigen::Vector2d& start,
             const Eigen::Vector2d& goal,
             const ObstacleArray& obstacles,
             double inflation = 0.2);

  /** @brief Get all nodes */
  const std::vector<VisibilityNode>& nodes() const { return nodes_; }

  /** @brief Get adjacency list for a node */
  const std::vector<VisibilityEdge>& edges(int node_id) const { return adj_[node_id]; }

  int startId() const { return start_id_; }
  int goalId() const  { return goal_id_; }
  bool empty() const  { return nodes_.empty(); }

private:
  /**
   * @brief Check if line p1→p2 is free of obstacle intersections
   */
  bool isVisible(const Eigen::Vector2d& p1,
                 const Eigen::Vector2d& p2,
                 const ObstacleArray& obstacles,
                 double inflation) const;

  /**
   * @brief Extract tangent keypoints around an obstacle polygon
   */
  std::vector<Eigen::Vector2d> extractKeypoints(
    const costmap_converter_msgs::msg::ObstacleMsg& obs,
    double inflation) const;

  std::vector<VisibilityNode> nodes_;
  std::vector<std::vector<VisibilityEdge>> adj_;
  int start_id_ = -1;
  int goal_id_  = -1;
};

} // namespace nav2_teb_controller
