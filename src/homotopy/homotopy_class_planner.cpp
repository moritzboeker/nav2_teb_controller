#include "nav2_teb_controller/homotopy/homotopy_class_planner.hpp"

namespace nav2_teb_controller {

HomotopyClassPlanner::HomotopyClassPlanner(
  const teb_controller::Params& params,
  const Footprint& footprint,
  nav2_costmap_2d::Costmap2DROS* costmap_ros) 
  : params_(params)
  , footprint_(footprint)
  , costmap_ros_(costmap_ros)
  , best_candidate_idx_(-1)
{
  RCLCPP_INFO(rclcpp::get_logger("HomotopyClassPlanner"), 
    "HomotopyClassPlanner constructed.");
}

bool HomotopyClassPlanner::plan(
  const nav_msgs::msg::Path& /*global_plan*/,
  const geometry_msgs::msg::Twist& /*start_vel*/)
{
  // TODO: Schritt 4
  return false;
}

bool HomotopyClassPlanner::hasDiverged()
{
  // TODO: Schritt 5
  return false;
}

void HomotopyClassPlanner::clear()
{
  candidates_.clear();
  best_candidate_idx_ = -1;
}

const TimedElasticBand& HomotopyClassPlanner::getTEB() const
{
  return getBestCandidate().teb;
}

void HomotopyClassPlanner::setFeedback(
  const ackermann_msgs::msg::AckermannDrive& feedback)
{
  if (base_planner_) base_planner_->setFeedback(feedback);
}

void HomotopyClassPlanner::updateObstacleContainer(
  costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obstacle_array)
{
  obstacles_ = obstacle_array;
  if (base_planner_) base_planner_->updateObstacleContainer(obstacle_array);
  if (graph_search_) graph_search_->updateObstacles(*obstacle_array);
}

void HomotopyClassPlanner::setObstacleMap(const ObstacleMap2D* esdf)
{ 
  if (base_planner_) base_planner_->setObstacleMap(esdf);
}

void HomotopyClassPlanner::setGraphSearch(std::shared_ptr<GraphSearchInterface> graph_search)
{
  graph_search_ = std::move(graph_search);
}

void HomotopyClassPlanner::setBasePlanner(
  std::shared_ptr<PlannerInterface<TimedElasticBand>> base_planner)
{
  base_planner_ = std::move(base_planner);
}

const TebCandidate& HomotopyClassPlanner::getBestCandidate() const
{
  if (candidates_.empty() || best_candidate_idx_ < 0)
    throw std::runtime_error("HCP: No valid candidate!");
  return *candidates_[best_candidate_idx_];
}

} // namespace nav2_teb_controller
