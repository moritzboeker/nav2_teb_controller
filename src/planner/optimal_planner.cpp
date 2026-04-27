#include "nav2_teb_controller/planner/optimal_planner.hpp"

namespace nav2_teb_controller {

DiscreteTEBPlanner::DiscreteTEBPlanner(
  const teb_controller::Params& params,
  const Footprint& footprint,
  nav2_costmap_2d::Costmap2DROS* costmap_ros) : params_(params), footprint_(footprint), costmap_ros_(costmap_ros)
{
  optimizer_ = initOptimizer();
  // obstacles_ = obstacles; // implement later
  // via_points_ = via_points;  // implement later
  cost_ = std::numeric_limits<double>::max();
  // prefer_rotdir_ = RotType::none;  // implement later

  vel_start_ = {false, geometry_msgs::msg::Twist()};
  vel_goal_  = {false, geometry_msgs::msg::Twist()};

  initialized_ = true;
}

std::shared_ptr<g2o::SparseOptimizer> DiscreteTEBPlanner::initOptimizer()
{
  // Parameter aus Config
  std::string solver_type = params_.FollowPath.optimizer.solver;
  std::string algorithm_type = params_.FollowPath.optimizer.algorithm;
  
  // Linear Solver Factory
  std::unique_ptr<g2o::BlockSolverX> block_solver;
  
  if (solver_type == "cholmod"){
    auto linear_solver = std::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();
    linear_solver->setBlockOrdering(true);
    block_solver = std::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
  } else if (solver_type == "eigen") {
    auto linear_solver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
    linear_solver->setBlockOrdering(true);
    block_solver = std::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
  } else if (solver_type == "csparse") {
    auto linear_solver = std::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
    linear_solver->setBlockOrdering(true);
    block_solver = std::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
  } else if (solver_type == "dense") {
    auto linear_solver = std::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
    block_solver = std::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("optimal_planner"),
      "Unknown solver '%s' - fallback to cholmod", solver_type.c_str());
    auto linear_solver = std::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();
    linear_solver->setBlockOrdering(true);
    block_solver = std::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
  }

  // Algorithm Factory
  std::unique_ptr<g2o::OptimizationAlgorithm> algorithm;
  if (algorithm_type == "gauss_newton") {
    algorithm = std::make_unique<g2o::OptimizationAlgorithmGaussNewton>(std::move(block_solver));
  } else if (algorithm_type == "levenberg_marquardt") {
    algorithm = std::make_unique<g2o::OptimizationAlgorithmLevenberg>(std::move(block_solver));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("optimal_planner"),
      "Unknown algorithm '%s' - fallback to lm", algorithm_type.c_str());
    algorithm = std::make_unique<g2o::OptimizationAlgorithmLevenberg>(std::move(block_solver));
  }
  
  std::shared_ptr<g2o::SparseOptimizer> optimizer = std::make_shared<g2o::SparseOptimizer>();
  optimizer->setAlgorithm(algorithm.release());
  optimizer->initMultiThreading();
  
  RCLCPP_INFO(rclcpp::get_logger("optimal_planner"), 
    "Optimizer configured: %s + %s", solver_type.c_str(), algorithm_type.c_str());
  return optimizer;
}

bool DiscreteTEBPlanner::plan(
  const nav_msgs::msg::Path& initial_plan,
  const geometry_msgs::msg::Twist& start_vel)
{
  if (!initialized_)
    return false;
  if (initial_plan.poses.empty())
    return false;

  // read params
  double v_max_x = params_.FollowPath.robot.v_max_x;
  double v_max_theta = params_.FollowPath.robot.v_max_theta;
  double reinit_dist = params_.FollowPath.trajectory.reinit_dist;
  double reinit_angle = params_.FollowPath.trajectory.reinit_angle;
  int min_samples = params_.FollowPath.trajectory.min_samples;
  int no_inner_iterations = params_.FollowPath.optimizer.no_inner_iterations;
  int no_outer_iterations = params_.FollowPath.optimizer.no_outer_iterations;
  bool overwrite_plan_orientation = params_.FollowPath.trajectory.overwrite_plan_orientation; 
  bool allow_init_backward = params_.FollowPath.trajectory.allow_init_backward;
  bool free_goal_vel = params_.FollowPath.optimizer.free_goal_vel;
  bool final_goal = params_.FollowPath.optimizer.fix_goal;

  if (!teb_.isInit())
  {
    // Cold start
    RCLCPP_INFO(rclcpp::get_logger("optimal_planner"), 
        "DiscreteTEBPlanner: Initialize new trajectory.");
    initFromPath(teb_, initial_plan, v_max_x, v_max_theta, 
      overwrite_plan_orientation, min_samples, allow_init_backward, final_goal);
  }
  else
  {
    // Warm start: check if goal has changed significantly
    const PoseSE2 start{initial_plan.poses.front().pose};
    const PoseSE2 goal{initial_plan.poses.back().pose};
    const double dist_to_goal = (goal.position() - teb_.backPose().position()).norm();
    const double angle_to_goal = std::abs(angles::normalize_angle( goal.theta() - teb_.backPose().theta()));
    const bool goal_changed = dist_to_goal > reinit_dist || angle_to_goal > reinit_angle;

    if (teb_.sizePoses() > 0 && !goal_changed)
      updateAndPrune(teb_, start, goal, min_samples);  // actual warm start
    else
    {
      // Goal too far away → reinit
      RCLCPP_INFO(rclcpp::get_logger("optimal_planner"), 
        "DiscreteTEBPlanner: Goal to far away. Initialize new trajectory.");
      teb_.clear();
      initFromPath(teb_, initial_plan, v_max_x, v_max_theta,
        overwrite_plan_orientation, min_samples, allow_init_backward, final_goal);
    }
  }
  // Set start velocity
  setVelocityStart(start_vel);
  // Goal velocity = zero (fixed)
  vel_goal_.first  = !free_goal_vel;
  vel_goal_.second = geometry_msgs::msg::Twist();

  return optimizeTEB(no_inner_iterations, no_outer_iterations, false);
}

void DiscreteTEBPlanner::clear()
{
  clearGraph();
  teb_.clear();
}

bool DiscreteTEBPlanner::optimizeTEB(int no_inner_iterations, int no_outer_iterations, bool compute_cost_afterwards)
{
  if (!params_.FollowPath.optimizer.activate)
    return false;
  
  bool success = false;
  weight_multiplier_ = 1.0;
  double chi2_old_ = INFINITY;
  optimized_ = false;

  // read params
  bool fast_mode = params_.FollowPath.optimizer.fast_mode;
  bool auto_resize = params_.FollowPath.trajectory.auto_resize;
  double dt_ref = params_.FollowPath.trajectory.dt_ref;
  double dt_hyst = params_.FollowPath.trajectory.dt_hyst;
  int min_samples = params_.FollowPath.trajectory.min_samples;
  int max_samples = params_.FollowPath.trajectory.max_samples;
  double min_seg_length = params_.FollowPath.trajectory.min_seg_length;
  double max_seg_length = params_.FollowPath.trajectory.max_seg_length;
  double max_angle_diff = params_.FollowPath.trajectory.max_angle_diff;
  double adapt_factor = params_.FollowPath.weights.weight_adapt_factor;

  if (auto_resize)
    autoResize(teb_, dt_ref, dt_hyst, min_seg_length, max_seg_length, max_angle_diff,
        min_samples, max_samples, fast_mode);

  success = buildGraph();
  if (!success)
  {
    RCLCPP_INFO(rclcpp::get_logger("optimal_planner"), "Building graph failed.");
    clearGraph();
    return false;
  }

  for (int i = 0; i < no_outer_iterations; ++i)
  {
    success = optimizeGraph(no_inner_iterations, true);
    if (!success)
    {
      RCLCPP_INFO(rclcpp::get_logger("optimal_planner"), "Optimizing graph failed.");
      clearGraph();
      return false;
    }

    double chi2_current = optimizer_->chi2();
    optimized_ = (chi2_old_ - chi2_current < 0.001);
    chi2_old_ = chi2_current;

    if (i < no_outer_iterations - 1)
      weight_multiplier_ *= adapt_factor;
  }

  if (compute_cost_afterwards)
    computeCurrentCost();
    
  writeBackOptimizedValues();
  clearGraph();

  RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"), "DiscreteTEBPlanner: TEB optimized.");
  
  /*
  for(int i=0; i<no_outer_iterations; ++i)
  {
    if (auto_resize)
      // autoResize(teb_, dt_ref, dt_hyst, min_samples, max_samples, fast_mode);
      autoResize(teb_, dt_ref, dt_hyst, min_seg_length,max_seg_length, max_angle_diff,
         min_samples, max_samples, fast_mode);

    success = buildGraph();
    if (!success) 
    {
      RCLCPP_ERROR(rclcpp::get_logger("optimal_planner"), 
        "DiscreteTEBPlanner: Building graph failed.");
      clearGraph();
      return false;
    }
    success = optimizeGraph(no_inner_iterations, false);
    if (!success) 
    {
      RCLCPP_ERROR(rclcpp::get_logger("optimal_planner"), 
        "DiscreteTEBPlanner: Optimizing graph failed.");
      clearGraph();
      return false;
    }
    double chi2_current = optimizer_->chi2();
    optimized_ = (chi2_old_ - chi2_current < 0.001);
    chi2_old_ = chi2_current;
    
    // optimized_ = true;
    
    // compute cost vec only in the last iteration
    if (compute_cost_afterwards && i==no_outer_iterations-1)
      computeCurrentCost();
      
    writeBackOptimizedValues();  // ← NEU: Werte zurück in teb_
    clearGraph();

    RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"), 
        "DiscreteTEBPlanner: TEB optimized.");
  
    weight_multiplier_ *= adapt_factor;
  }
  */
  return true;
}

bool DiscreteTEBPlanner::buildGraph()
{
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
    return false;

  // read params
  bool divergence_detection = params_.FollowPath.recovery.divergence_detection_enable;
  std::string robot_model = params_.FollowPath.robot.robot_model;
  RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"), 
        "DiscreteTEBPlanner: Build graph.");
  optimizer_->setComputeBatchStatistics(divergence_detection);
  
  // add TEB vertices
  AddTEBVertices();
  RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"), 
        "DiscreteTEBPlanner: Added vertices.");

  const bool holonomic = params_.FollowPath.robot.v_max_y > 0;

  // Edges
  // Generic Edges
  // AddEdgesVelocity();
  if (!holonomic)
    addEdgesGeneric<EdgeVelocity, 2>(
      {2, 1, 1, 0, 2},
      {params_.FollowPath.weights.weight_v_max_x,
        params_.FollowPath.weights.weight_v_max_theta});
  else
    addEdgesGeneric<EdgeVelocityHolonomic, 3>(
      {2, 1, 1, 0, 3},
      {params_.FollowPath.weights.weight_v_max_x,
        params_.FollowPath.weights.weight_v_max_y,
        params_.FollowPath.weights.weight_v_max_theta});
  // AddEdgesTimeOptimal();
  addEdgesGeneric<EdgeTimeOptimal, 1>(
    {0, 1, 1, 0, 1},
    {params_.FollowPath.weights.weight_time_optimal});
  // AddEdgesShortestPath();
  addEdgesGeneric<EdgeShortestPath, 1>(
    {2, 0, 1, 0, 1},
    {params_.FollowPath.weights.weight_shortest_path});
  // AddEdgesPathSmoothness();
  addEdgesGeneric<EdgePathSmoothness, 1>(
    {2, 0, 1, 0, 1},
    {params_.FollowPath.weights.weight_shortest_path});
  // addEdgesGeneric<EdgeJerk, 2>(
  //     {4, 3, 3, 0, 2},
  //     {params_.FollowPath.weights.weight_jerk_max_x,
  //     params_.FollowPath.weights.weight_jerk_max_theta});
  // AddEdgesSnap();
  addEdgesGeneric<EdgeSnap, 2>(
      {5, 4, 4, 0, 2},
      {params_.FollowPath.weights.weight_snap_max_x,
      params_.FollowPath.weights.weight_snap_max_theta});  
  // AddEdgesG3Continuity();
  addEdgesGeneric<EdgeG3Continuity, 1>(
    {3, 0, 2, 0, 1},
    {params_.FollowPath.weights.weight_g3_continuity});
  // AddEdgesSteeringAngleGoal();
  addEdgesGeneric<EdgeSteeringAngleGoal, 1>(
    {5, 0, 0, -1, 1},
    {params_.FollowPath.weights.weight_zero_steering_angle_goal});
  // AddEdgesGoalAngularVelocityZero();
  addEdgesGeneric<EdgeGoalAngularVelocityZero, 1>(
    {5, 4, 0, -1, 1},
    {params_.FollowPath.weights.weight_goal_angular_vel_zero});

  // Kinematic Edges
  if (robot_model == "diff_drive")
    // AddEdgesKinematicsDiffDrive();
    addEdgesGeneric<EdgeKinematicsDiffDrive, 2>(
      {2, 0, 1, 0, 2},
      {params_.FollowPath.weights.weight_kinematics_nh,
      params_.FollowPath.weights.weight_kinematics_forward_drive});
  else if (robot_model == "ackermann")
    // AddEdgesKinematicsCarlike();
    addEdgesGeneric<EdgeKinematicsCarlike, 2>(
      {2, 0, 1, 0, 2},
      {params_.FollowPath.weights.weight_kinematics_nh,
      params_.FollowPath.weights.weight_kinematics_turning_radius});

  RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"), 
        "DiscreteTEBPlanner: Added generic edges.");

  // Add Edges with measurement
  AddEdgesAcceleration();
  AddEdgesSteeringRate();
  AddEdgesStartSteeringAngle();
  AddEdgesJerk();

  RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"), 
        "DiscreteTEBPlanner: Added measurement edges.");
  
  // Via points
  AddEdgesViaPoints();

  // Obstacle Edges
  // AddEdgesObstacles();
  addEdgesESDFObstacles();

  RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"), 
        "DiscreteTEBPlanner: Added obstacle edges.");

  // Recovery
  AddEdgesPreferRotDir();

  RCLCPP_DEBUG(rclcpp::get_logger("optimal_planner"), 
        "DiscreteTEBPlanner: Added edges.");

  optimizer_->initializeOptimization();
  return true;
  /*
  auto safe_add = [&](const char* name, auto&& fn) -> bool 
  {
    try {
      fn();
      RCLCPP_INFO(rclcpp::get_logger("optimal_planner"),
        "DiscreteTEBPlanner: AddEdges OK: %s", name);
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("optimal_planner"),
        "DiscreteTEBPlanner: AddEdges CRASHED in %s: %s", name, e.what());
      return false;
    } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("optimal_planner"),
        "DiscreteTEBPlanner: AddEdges CRASHED in %s: unknown exception", name);
      return false;
    }
  };

  bool ok = true;
  ok &= safe_add("Velocity", [&]{ AddEdgesVelocity(); });
  ok &= safe_add("Acceleration", [&]{ AddEdgesAcceleration(); });
  ok &= safe_add("TimeOptimal", [&]{ AddEdgesTimeOptimal(); });
  ok &= safe_add("ShortestPath", [&]{ AddEdgesShortestPath(); });
  ok &= safe_add("PathSmoothness", [&]{ AddEdgesPathSmoothness(); });
  ok &= safe_add("ViaPoints", [&]{ AddEdgesViaPoints(); });
  ok &= safe_add("Jerk", [&]{ AddEdgesJerk(); });
  ok &= safe_add("Snap", [&]{ AddEdgesSnap(); });
  ok &= safe_add("SteeringAngleGoal", [&]{ AddEdgesSteeringAngleGoal(); });
  ok &= safe_add("SteeringRate", [&]{ AddEdgesSteeringRate(); });
  ok &= safe_add("GoalAngularVelocityZero", [&]{ AddEdgesGoalAngularVelocityZero(); });
  ok &= safe_add("TurningRadiusChange", [&]{ AddEdgesTurningRadiusChange(); });
  ok &= safe_add("StartSteeringAngle", [&]{ AddEdgesStartSteeringAngle(); });
  ok &= safe_add("G3Continuity", [&]{ AddEdgesG3Continuity(); });

  if (robot_model == "diff_drive")
    ok &= safe_add("KinematicsDiffDrive", [&]{ AddEdgesKinematicsDiffDrive(); });
  else if (robot_model == "ackermann")
    ok &= safe_add("KinematicsCarlike", [&]{ AddEdgesKinematicsCarlike(); });

  if (legacy_obstacle_association)
    ok &= safe_add("ObstaclesLegacy", [&]{ AddEdgesObstaclesLegacy(); });
  else
    ok &= safe_add("Obstacles", [&]{ AddEdgesObstacles(); });

  if (include_dynamic_obstacles)
    ok &= safe_add("DynamicObstacles", [&]{ AddEdgesDynamicObstacles(); });

  ok &= safe_add("VelocityObstacleRatio", [&]{ AddEdgesVelocityObstacleRatio(); });
  ok &= safe_add("PreferRotDir", [&]{ AddEdgesPreferRotDir(); });

  if (!ok) return false;
  */
}

bool DiscreteTEBPlanner::optimizeGraph(int no_inner_iterations, bool clear_after)
{
  // read params
  double v_max_x = params_.FollowPath.robot.v_max_x;
  int min_samples = params_.FollowPath.trajectory.min_samples;
  bool verbose = params_.FollowPath.optimizer.verbose;

  // Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted.
  if (v_max_x < 0.01)
  {
    RCLCPP_INFO(rclcpp::get_logger("optimal_planner"), 
      "vmax too small.");
    if (clear_after)
      clearGraph();
    return false;	
  }
  
  // TEB is empty or has too less elements. Skipping optimization.
  if (!teb_.isInit() || teb_.sizePoses() < static_cast<std::size_t>(min_samples))
  {
    RCLCPP_INFO(rclcpp::get_logger("optimal_planner"), 
      "teb no initialized.");
    if (clear_after)
      clearGraph();
    return false;	
  }
  
  optimizer_->setVerbose(verbose);
  // optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_inner_iterations);

  // Optimization failed;
  if (iter == 0) 
  {
    RCLCPP_INFO(rclcpp::get_logger("optimal_planner"), 
      "Optimization failed.");
    return false;
  }

  if (clear_after)
    clearGraph();	
    
  return true;
}

void DiscreteTEBPlanner::clearGraph()
{
  // clear optimizer states
  if (optimizer_)
  {
    // we will delete all edges but keep the vertices.
    // before doing so, we will delete the link from the vertices to the edges.
    auto& vertices = optimizer_->vertices();
    for(auto& v : vertices)
      v.second->edges().clear();

    // necessary, because optimizer->clear deletes pointer-targets 
    // (therefore it deletes TEB states!)
    optimizer_->vertices().clear();
    optimizer_->clear();
  }
}

bool DiscreteTEBPlanner::hasDiverged()
{
  const bool enable = params_.FollowPath.optimizer.divergence_detection_enable;
  const double max_chi2 = params_.FollowPath.optimizer.divergence_detection_max_chi_squared;
  const double max_violation_rate = params_.FollowPath.optimizer.divergence_detection_max_chi_violation_rate;
  const double max_path_factor = params_.FollowPath.optimizer.divergence_detection_max_path_length_factor;
  const double dt_ref = params_.FollowPath.trajectory.dt_ref;
  const double max_samples = params_.FollowPath.trajectory.max_samples;
  const double v_max = params_.FollowPath.robot.v_max_x;
  
  if (!enable)
    return false;
  auto stats_vector = optimizer_->batchStatistics();
  if (stats_vector.empty()) // No statistics yet
    return false;
  const auto last_iter_stats = stats_vector.back();

  if (last_iter_stats.chi2 > max_chi2)
  {
    RCLCPP_INFO(rclcpp::get_logger("optimal_planner"), 
          "DiscreteTEBPlanner: Optimizer has diverged, max_chi_2.");
    return true;
  }

  int violated_edges = 0;
  for (const auto* edge : optimizer_->activeEdges())
    if (edge->chi2() > 1.0) ++violated_edges;
  const double violation_rate = static_cast<double>(violated_edges) / optimizer_->activeEdges().size();

  if (violation_rate > max_violation_rate )
  {
    RCLCPP_INFO(rclcpp::get_logger("optimal_planner"), 
          "DiscreteTEBPlanner: Optimizer has diverged, violation_rate.");
    return true;
  }

  const double path_length = teb_.accumulatedDistance();
  const double max_path_length = dt_ref * max_samples * v_max * max_path_factor;
  if (path_length > max_path_length)
  {
    RCLCPP_INFO(rclcpp::get_logger("optimal_planner"), 
          "DiscreteTEBPlanner: Optimizer has diverged, violation_rate.");
    return true;
  }
  return false;                       
}

void DiscreteTEBPlanner::computeCurrentCost()
{

}

void DiscreteTEBPlanner::writeBackOptimizedValues()
{
  for (std::size_t i = 0; i < pose_vertices_.size(); ++i)
  {
    teb_.pose(i) = pose_vertices_[i]->pose();
  }
  for (std::size_t i = 0; i < timediff_vertices_.size(); ++i)
  {
    teb_.timeDiff(i) = timediff_vertices_[i]->dt();
  }
}

void DiscreteTEBPlanner::updateObstacleContainer (costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obstacle_array)
{
  if (obstacle_array)
  {
    obstacles_.clear();
    for (const auto& obs : obstacle_array->obstacles)
    {
      if (obs.polygon.points.empty()) continue;
      std::vector<Eigen::Vector2d> polygon;
      polygon.reserve(obs.polygon.points.size());
      for (const auto& pt : obs.polygon.points)
        polygon.emplace_back(pt.x, pt.y);
      obstacles_.push_back({polygon, obs.radius});
    }
  }
  else
    extractObstacles();
}

void DiscreteTEBPlanner::AddTEBVertices()
{
  bool final_goal = params_.FollowPath.optimizer.fix_goal;

  pose_vertices_.clear();
  timediff_vertices_.clear();

  // obstacles_per_vertex_.resize(teb_.sizePoses());

  unsigned int id_counter = 0;

  for (std::size_t i = 0; i < teb_.sizePoses(); ++i)
  {
    // Start (i==0) und Goal (i==last) fixieren
    const bool fixed = (i == 0) || ((i == teb_.sizePoses() - 1) && final_goal);

    // Pose-Vertex aus PoseSE2 erstellen
    auto* pose_vtx = new VertexPose(teb_.pose(i), fixed);
    pose_vtx->setId(id_counter++);
    optimizer_->addVertex(pose_vtx);
    pose_vertices_.push_back(pose_vtx);

    // TimeDiff-Vertex erstellen (N-1 Stück)
    if (i < teb_.sizeTimeDiffs())
    {
      auto* dt_vtx = new VertexTimeDiff(teb_.timeDiff(i), false);
      dt_vtx->setId(id_counter++);
      optimizer_->addVertex(dt_vtx);
      timediff_vertices_.push_back(dt_vtx);
    }

    // obstacles_per_vertex_[i].clear();
    // obstacles_per_vertex_[i].reserve(obstacles_->size());
  }
}

// Start: 3 Vertex (2 Pose + 1 Time), first 2 poses, 2 weights, 2D, setInitialVelocity
// All:   5 Vertex (3 Pose + 2 Time), All poses-2, 2 weights, 2D
// Goal:  3 Vertex (2 Pose + 1 Time), last 2 poses, 2 weights, 2D, setGoalVelocity
// holonomic -> Start: 3 Vertex (2 Pose + 1 Time), first 2 poses, 3 weights, 3D, setInitialVelocity
// holonomic -> All:   5 Vertex (3 Pose + 2 Time), All poses -2, 3 weights, 3D
// holonomic -> Goal:  3 Vertex (2 Pose + 1 Time), last 2 poses, 3 weights, 3D, setGoalVelocity
void DiscreteTEBPlanner::AddEdgesAcceleration()
{
  const bool holonomic = params_.FollowPath.robot.v_max_y > 0;
  const double weight_x = params_.FollowPath.weights.weight_a_max_x;
  const double weight_y = params_.FollowPath.weights.weight_a_max_y;
  const double weight_theta = params_.FollowPath.weights.weight_a_max_theta;
  
  if (!holonomic) // non-holonomic robot
  {
    if (weight_x==0  && weight_theta==0) 
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();  
    Eigen::Matrix<double,2,2> information;
    information.fill(0);
    information(0,0) = weight_x;
    information(1,1) = weight_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0, pose_vertices_[0]);
      acceleration_edge->setVertex(1, pose_vertices_[1]);
      acceleration_edge->setVertex(2, timediff_vertices_[0]);
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(params_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0, pose_vertices_[i]);
      acceleration_edge->setVertex(1, pose_vertices_[i+1]);
      acceleration_edge->setVertex(2, pose_vertices_[i+2]);
      acceleration_edge->setVertex(3, timediff_vertices_[i]);
      acceleration_edge->setVertex(4, timediff_vertices_[i+1]);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(params_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0, pose_vertices_[n-2]);
      acceleration_edge->setVertex(1, pose_vertices_[n-1]);
      acceleration_edge->setVertex(2, timediff_vertices_[teb_.sizeTimeDiffs()-1]);
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(params_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
  else // holonomic robot
  {
    if (weight_x==0  && weight_y==0 && weight_theta==0) 
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = weight_x;
    information(1,1) = weight_y;
    information(2,2) = weight_theta;
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationHolonomicStart* acceleration_edge = new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0, pose_vertices_[0]);
      acceleration_edge->setVertex(1, pose_vertices_[1]);
      acceleration_edge->setVertex(2, timediff_vertices_[0]);
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(params_);
      optimizer_->addEdge(acceleration_edge);
    }
    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAccelerationHolonomic* acceleration_edge = new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0, pose_vertices_[i]);
      acceleration_edge->setVertex(1, pose_vertices_[i+1]);
      acceleration_edge->setVertex(2, pose_vertices_[i+2]);
      acceleration_edge->setVertex(3, timediff_vertices_[i]);
      acceleration_edge->setVertex(4, timediff_vertices_[i+1]);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(params_);
      optimizer_->addEdge(acceleration_edge);
    }
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0, pose_vertices_[n-2]);
      acceleration_edge->setVertex(1, pose_vertices_[n-1]);
      acceleration_edge->setVertex(2, timediff_vertices_[teb_.sizeTimeDiffs()-1]);
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(params_);
      optimizer_->addEdge(acceleration_edge);
    }
  }
}

void DiscreteTEBPlanner::AddEdgesJerk()
{
  const double weight_x = params_.FollowPath.weights.weight_jerk_max_x;
  const double weight_theta = params_.FollowPath.weights.weight_jerk_max_theta;
  
  if (weight_x==0  && weight_theta==0) 
    return; // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();

  // We need at least 4 poses
  if (n < 4)
      return;

  Eigen::Matrix<double,2,2> information;
  information.fill(0);
  information(0,0) = weight_x;
  information(1,1) = weight_theta;
    
  // check if an initial velocity should be taken into accound
  if (vel_start_.first)
  {
    EdgeJerkStart* jerk_edge = new EdgeJerkStart;
    jerk_edge->setVertex(0, pose_vertices_[0]);
    jerk_edge->setVertex(1, pose_vertices_[1]);
    jerk_edge->setVertex(2, pose_vertices_[2]);
    jerk_edge->setVertex(3, timediff_vertices_[0]);
    jerk_edge->setVertex(4, timediff_vertices_[1]);
    jerk_edge->setInitialVelocity(vel_start_.second);
    jerk_edge->setInformation(information);
    jerk_edge->setTebConfig(params_);
    optimizer_->addEdge(jerk_edge);
  }
  // now add the usual jerk edge for each tuple of three teb poses
  for (int i=0; i < n - 3; ++i)
  {
    EdgeJerk* jerk_edge = new EdgeJerk;
    jerk_edge->setVertex(0, pose_vertices_[i]);
    jerk_edge->setVertex(1, pose_vertices_[i+1]);
    jerk_edge->setVertex(2, pose_vertices_[i+2]);
    jerk_edge->setVertex(3, pose_vertices_[i+3]);
    jerk_edge->setVertex(4, timediff_vertices_[i]);
    jerk_edge->setVertex(5, timediff_vertices_[i+1]);
    jerk_edge->setVertex(6, timediff_vertices_[i+2]);
    jerk_edge->setInformation(information);
    jerk_edge->setTebConfig(params_);
    optimizer_->addEdge(jerk_edge);
  }
  // check if a goal velocity should be taken into accound
  if (vel_goal_.first)
  {
    EdgeJerkGoal* jerk_edge = new EdgeJerkGoal;
    jerk_edge->setVertex(0, pose_vertices_[n-3]);
    jerk_edge->setVertex(1, pose_vertices_[n-2]);
    jerk_edge->setVertex(2, pose_vertices_[n-1]);
    jerk_edge->setVertex(3, timediff_vertices_[teb_.sizeTimeDiffs()-2]);
    jerk_edge->setVertex(4, timediff_vertices_[teb_.sizeTimeDiffs()-1]);
    jerk_edge->setGoalVelocity(vel_goal_.second);
    jerk_edge->setInformation(information);
    jerk_edge->setTebConfig(params_);
    optimizer_->addEdge(jerk_edge);
  }  
}

// Start: 3 Vertex (2 Pose + 1 Time), first 2 poses, 1 weights, 1D, setInitialSteeringAngle
// All:   5 Vertex (3 Pose + 2 Time), All poses -2, 1 weights, 1D
// Goal:  3 Vertex (2 Pose + 1 Time), last 2 poses, 1 weights, 1D, setGoalSteeringAngle
void DiscreteTEBPlanner::AddEdgesSteeringRate()
{
  const double weight = params_.FollowPath.weights.weight_max_steering_rate;
  if (weight==0)
      return; // if weight equals zero skip adding edges!
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,1,1> information_steering_rate;
  information_steering_rate(0, 0) = weight;  
  int n = teb_.sizePoses();
  // check if an initial velocity should be taken into accound (we apply the same for the steering rate)
  if (vel_start_.first)
  {
    EdgeSteeringRateStart* steering_rate_edge = new EdgeSteeringRateStart;
    steering_rate_edge->setVertex(0, pose_vertices_[0]);
    steering_rate_edge->setVertex(1, pose_vertices_[1]);
    steering_rate_edge->setVertex(2, timediff_vertices_[0]);
    steering_rate_edge->setInitialSteeringAngle(ackermann_feedback_.steering_angle);
    steering_rate_edge->setInformation(information_steering_rate);
    steering_rate_edge->setTebConfig(params_);
    optimizer_->addEdge(steering_rate_edge);
  }
  for (int i=0; i < n-2; i++) // ignore twiced start only
  {
      EdgeSteeringRate* steering_rate_edge = new EdgeSteeringRate;
      steering_rate_edge->setVertex(0, pose_vertices_[i]);
      steering_rate_edge->setVertex(1, pose_vertices_[i+1]);      
      steering_rate_edge->setVertex(2, pose_vertices_[i+2]);   
      steering_rate_edge->setVertex(3, timediff_vertices_[i]); 
      steering_rate_edge->setVertex(4, timediff_vertices_[i+1]); 
      steering_rate_edge->setInformation(information_steering_rate);
      steering_rate_edge->setTebConfig(params_);
      // optimizer_->addEdge(steering_rate_edge);
  }
  // check if a goal velocity should be taken into accound (we apply the same for the steering rate)
  if (vel_goal_.first)
  {
    EdgeSteeringRateGoal* steering_rate_edge = new EdgeSteeringRateGoal;
    steering_rate_edge->setVertex(0, pose_vertices_[n-2]);
    steering_rate_edge->setVertex(1, pose_vertices_[n-1]);
    steering_rate_edge->setVertex(2, timediff_vertices_[teb_.sizeTimeDiffs()-1]);
    steering_rate_edge->setGoalSteeringAngle(0.0);
    steering_rate_edge->setInformation(information_steering_rate);
    steering_rate_edge->setTebConfig(params_);
    optimizer_->addEdge(steering_rate_edge);
  }  
}

// 5 Vertex (3 Pose + 2 Time), first 3 poses, 1 weights, 2D, setInitialSteeringAngle
void DiscreteTEBPlanner::AddEdgesStartSteeringAngle()
{
  const double weight = params_.FollowPath.weights.weight_start_steering_angle;
  if (weight==0)
      return; // if weight equals zero skip adding edges!

  // We need at least 3 poses
  if (teb_.sizePoses() < 3)
      return;

  // Create the information matrix (weight).
  // Since our error is 2D, the information matrix is 2x2.
  Eigen::Matrix<double, 2, 2> information = Eigen::Matrix<double, 2, 2>::Identity() * weight;
  EdgeStartSteeringAngle* start_edge = new EdgeStartSteeringAngle();
  start_edge->setVertex(0, pose_vertices_[0]);
  start_edge->setVertex(1, pose_vertices_[1]);
  start_edge->setVertex(2, pose_vertices_[2]);
  start_edge->setVertex(3, timediff_vertices_[0]);
  start_edge->setVertex(4, timediff_vertices_[1]);
  start_edge->setInitialSteeringAngle(ackermann_feedback_.steering_angle);
  start_edge->setInformation(information);
  start_edge->setTebConfig(params_);
  optimizer_->addEdge(start_edge);
}

void DiscreteTEBPlanner::extractObstacles()
{
  obstacles_.clear();
  auto* costmap = costmap_ros_->getCostmap();
  
  // Nur LETHAL-Zellen → Punkt-Obstacles
  for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
    for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
      if (costmap->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        double wx, wy;
        costmap->mapToWorld(mx, my, wx, wy);
        obstacles_.push_back({{Eigen::Vector2d(wx, wy)}, 0.0});
      }
    }
  }
}

void DiscreteTEBPlanner::AddEdgesObstacles()
{
  const double weight_obstacle = params_.FollowPath.weights.weight_obstacle;
  const double weight_inflation = params_.FollowPath.weights.weight_inflation;
  const double min_dist = params_.FollowPath.obstacles.min_obstacle_dist;
  const double cutoff_buffer = params_.FollowPath.obstacles.cutoff_dist;
  const auto& raw_fp = costmap_ros_->getRobotFootprint();
  const double circum_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  double cutoff = circum_radius + min_dist + cutoff_buffer;

  if (weight_obstacle == 0.0 && weight_inflation == 0.0) return;

  Eigen::Matrix<double,2,2> information;
  information.fill(0);
  information(0,0) = weight_obstacle * weight_multiplier_;
  information(1,1) = weight_inflation;

  for (std::size_t i = 0; i < teb_.sizePoses(); ++i)
  {
    const Eigen::Vector2d pose_pos = teb_.pose(i).position();

    for (const auto& obs : obstacles_)
    {
      // Obstacle (grober Vorfilter), check closest polygon point
      double rough_dist = std::numeric_limits<double>::max();
      for (const auto& pt : obs.polygon)
      {
        if (rough_dist > (pose_pos - pt).norm())
          rough_dist = (pose_pos - pt).norm();
      }
      if (rough_dist > cutoff) continue;  // zu weit weg → skip

      EdgeCostmapObstacle* e = new EdgeCostmapObstacle();
      e->setVertex(0, pose_vertices_[i]);
      e->setObstacle(obs);
      e->setFootprint(raw_fp);
      e->setInformation(information);
      e->setTebConfig(params_);
      optimizer_->addEdge(e);
    }
  }
}

void DiscreteTEBPlanner::addEdgesESDFObstacles()
{
  const double weight_obstacle  = params_.FollowPath.weights.weight_obstacle;
  const double weight_inflation = params_.FollowPath.weights.weight_inflation;

  if (weight_obstacle == 0.0 && weight_inflation == 0.0) return;

  const int n_circles = static_cast<int>(footprint_.circles().size());
  const int dim       = n_circles + 1;

  // Information matrix — dynamisch, zur Laufzeit aufgebaut
  Eigen::MatrixXd information = Eigen::MatrixXd::Zero(dim, dim);
  for (int i = 0; i < n_circles; ++i)
    information(i, i) = weight_obstacle * weight_multiplier_;
  information(dim - 1, dim - 1) = weight_inflation;

  for (std::size_t i = 0; i < teb_.sizePoses(); ++i) {
    auto* e = new EdgeESDFObstacle();
    e->resize(n_circles);          // ← setzt _dimension vor addEdge
    e->setVertex(0, pose_vertices_[i]);
    e->setObstacle(*esdf_);
    e->setFootprint(footprint_);
    e->setTebConfig(params_);
    e->setInformation(information);
    optimizer_->addEdge(e);
  }
}

void DiscreteTEBPlanner::AddEdgesViaPoints(){}
void DiscreteTEBPlanner::AddEdgesPreferRotDir(){}

template<typename EdgeType, int InfoDim, typename... Args>
void DiscreteTEBPlanner::addEdgesGeneric(const EdgeDescriptor& desc, 
                     const std::array<double, InfoDim>& weights,
                     Args&&... args)
{
  // Prüfe ob alle weights > 0
  for (double w : weights)
    if (w == 0.0) return;

  const int n = static_cast<int>(teb_.sizePoses());

  if ((desc.num_poses && (n < desc.num_poses))
    || (desc.num_timediffs && (n-1 < desc.num_timediffs)))
    return;

  // Iterationsbereich bestimmen
  int start_i, end_i;
  if (desc.offset >= 0) {
    // Von vorne: z.B. TimeOptimal, Jerk, StartSteering
    start_i = desc.offset;
    end_i   = (desc.stride == 0)
              ? start_i + 1                          // einmalig (Start/Goal-Edges)
              : n - desc.stride;                     // alle - stride
  } else {
    // Von hinten
    if (desc.stride == 0) {
      // Einmalig: genau die letzten num_poses Posen
      start_i = n - desc.num_poses;   // 5-5=0 ✓
      end_i   = start_i + 1;          // 1 ✓
    } else {
      // Sliding bis vor das Ende
      start_i = 0;
      end_i   = n - desc.num_poses - std::abs(desc.offset) + 1;
    }
  }
  // Information-Matrix aufbauen
  Eigen::Matrix<double, InfoDim, InfoDim> info =
    Eigen::Matrix<double, InfoDim, InfoDim>::Zero();
  for (int d = 0; d < InfoDim; ++d)
    info(d, d) = weights[d];

  for (int i = start_i; i < end_i; ++i)
  {
    auto* e = new EdgeType();
    // Pose-Vertices setzen
    for (int p = 0; p < desc.num_poses; ++p)
      e->setVertex(p, pose_vertices_[i + p]);

    // TimeDiff-Vertices setzen (nach den Pose-Vertices)
    for (int t = 0; t < desc.num_timediffs; ++t)
      e->setVertex(desc.num_poses + t, timediff_vertices_[i + t]);

    e->setInformation(info);
    e->setTebConfig(params_);
    optimizer_->addEdge(e);
  }
}

} // namespace nav2_teb_controller


// replaced with addEdgesGeneric
/*
// 1 Vertex Time, All timediff, 1 weight, 1D
void DiscreteTEBPlanner::AddEdgesTimeOptimal() 
{
  const double weight = params_.FollowPath.weights.weight_time_optimal;
  if (weight==0) return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(weight);
  for (std::size_t i=0; i < teb_.sizeTimeDiffs(); ++i)
  {
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0, timediff_vertices_[i]);
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setTebConfig(params_);
    optimizer_->addEdge(timeoptimal_edge);
  }
  // EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
  // VertexTimeDiff* sum_time_diff = new VertexTimeDiff(teb_.getSumOfAllTimeDiffs());
  // timeoptimal_edge->setVertex(0, sum_time_diff);
  // timeoptimal_edge->setInformation(information);
  // timeoptimal_edge->setTebConfig(*cfg_);
  // optimizer_->addEdge(timeoptimal_edge);
}
*/
/*
// 2 Vertex Pose, All poses-1, 1 weight, 1D
void DiscreteTEBPlanner::AddEdgesShortestPath()
{
  const double weight = params_.FollowPath.weights.weight_shortest_path;
  if (weight==0) return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(weight);

  for (std::size_t i=0; i < teb_.sizePoses()-1; ++i)
  {
    EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0, pose_vertices_[i]);
    shortest_path_edge->setVertex(1, pose_vertices_[i+1]);
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setTebConfig(params_);
    optimizer_->addEdge(shortest_path_edge);
  }
}
*/
/*
// 2 Vertex Pose, All poses-1, 1 weight, 1D
void DiscreteTEBPlanner::AddEdgesPathSmoothness()
{
  const double weight = params_.FollowPath.weights.weight_path_smoothness;
  if (weight==0) return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 1, 1> information;
  information.fill(weight);

  for (std::size_t i=0; i < teb_.sizePoses()-1; ++i)
  {
    EdgePathSmoothness *smoothness_edge = new EdgePathSmoothness;
    smoothness_edge->setVertex(0, pose_vertices_[i]);
    smoothness_edge->setVertex(1, pose_vertices_[i+1]);
    smoothness_edge->setInformation(information);
    smoothness_edge->setTebConfig(params_);
    optimizer_->addEdge(smoothness_edge);
  }
}
*/
/*
// 3 Vertex (2 Pose + 1 Time), All poses-1, 2 weights, 2D
// holonomic -> 3 Vertex (2 Pose + 1 Time), All poses-1, 3 weights, 3D
void DiscreteTEBPlanner::AddEdgesVelocity()
{
  const bool holonomic = params_.FollowPath.robot.v_max_y > 0;
  const double weight_x = params_.FollowPath.weights.weight_v_max_x;
  const double weight_y = params_.FollowPath.weights.weight_v_max_y;
  const double weight_theta = params_.FollowPath.weights.weight_v_max_theta;

  if (!holonomic) // non-holonomic robot
  {
    if (weight_x==0 && weight_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double,2,2> information;
    information(0,0) = weight_x;
    information(1,1) = weight_theta;
    information(0,1) = 0.0;
    information(1,0) = 0.0;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0,pose_vertices_[i]);
      velocity_edge->setVertex(1,pose_vertices_[i+1]);
      velocity_edge->setVertex(2,timediff_vertices_[i]);
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(params_);
      optimizer_->addEdge(velocity_edge);
    }
  }
  else // holonomic-robot
  {
    if (weight_x==0 && weight_y==0 && weight_theta==0)
      return; // if weight equals zero skip adding edges!
      
    int n = teb_.sizePoses();
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = weight_x;
    information(1,1) = weight_y;
    information(2,2) = weight_theta;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0,pose_vertices_[i]);
      velocity_edge->setVertex(1,pose_vertices_[i+1]);
      velocity_edge->setVertex(2,timediff_vertices_[i]);
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(params_);
      optimizer_->addEdge(velocity_edge);
    } 
  }
}
*/
/*
// 7 Vertex (4 Pose + 3 Time), All poses -3, 2 weights, 2D
void DiscreteTEBPlanner::AddEdgesJerk()
{
  const double weight_x = params_.FollowPath.weights.weight_jerk_max_x;
  const double weight_theta = params_.FollowPath.weights.weight_jerk_max_theta;
  if (weight_x == 0 && weight_theta == 0)
      return;

  Eigen::Matrix<double, 2, 2> information;
  information.setZero();
  information(0, 0) = weight_x;
  information(1, 1) = weight_theta;

  // The edge connects 4 poses, so we must stop 3 poses before the end.
  for (std::size_t i = 0; i < teb_.sizePoses() - 3; ++i)
  {
      EdgeJerk* jerk_edge = new EdgeJerk();
      // Connect the 7 vertices: 4 poses and 3 time diffs
      jerk_edge->setVertex(0, pose_vertices_[i]);
      jerk_edge->setVertex(1, pose_vertices_[i+1]);
      jerk_edge->setVertex(2, pose_vertices_[i+2]);
      jerk_edge->setVertex(3, pose_vertices_[i+3]);
      jerk_edge->setVertex(4, timediff_vertices_[i]);
      jerk_edge->setVertex(5, timediff_vertices_[i+1]);
      jerk_edge->setVertex(6, timediff_vertices_[i+2]);
      jerk_edge->setInformation(information);
      jerk_edge->setTebConfig(params_);
      optimizer_->addEdge(jerk_edge);
  }
}
*/
/*
// 9 Vertex (5 Pose + 4 Time), All poses -4, 2 weights, 2D
void DiscreteTEBPlanner::AddEdgesSnap()
{
  const double weight_x = params_.FollowPath.weights.weight_snap_max_x;
  const double weight_theta = params_.FollowPath.weights.weight_snap_max_theta;
  if (weight_x == 0 && weight_theta == 0)
      return;

  Eigen::Matrix<double, 2, 2> information;
  information.setZero();
  information(0, 0) = weight_x;
  information(1, 1) = weight_theta;

  // The edge connects 5 poses, so we must stop 4 poses before the end.
  for (std::size_t i = 0; i < teb_.sizePoses() - 4; ++i)
  {
      EdgeSnap* snap_edge = new EdgeSnap();
      // Connect the 9 vertices: 5 poses and 4 time diffs
      snap_edge->setVertex(0, pose_vertices_[i]);
      snap_edge->setVertex(1, pose_vertices_[i+1]);
      snap_edge->setVertex(2, pose_vertices_[i+2]);
      snap_edge->setVertex(3, pose_vertices_[i+3]);
      snap_edge->setVertex(4, pose_vertices_[i+4]);
      snap_edge->setVertex(5, timediff_vertices_[i]);
      snap_edge->setVertex(6, timediff_vertices_[i+1]);
      snap_edge->setVertex(7, timediff_vertices_[i+2]);
      snap_edge->setVertex(8, timediff_vertices_[i+3]);
      snap_edge->setInformation(information);
      snap_edge->setTebConfig(params_);
      optimizer_->addEdge(snap_edge);
  }
}
*/
/*
// 2 Vertex Pose, All poses -1, 2 weights, 2D
void DiscreteTEBPlanner::AddEdgesKinematicsDiffDrive()
{
  const double weight_nh = params_.FollowPath.weights.weight_kinematics_nh;
  const double weight_forward = params_.FollowPath.weights.weight_kinematics_forward_drive;
  if (weight_nh==0 && weight_forward==0)
    return; // if weight equals zero skip adding edges!
  
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = weight_nh;
  information_kinematics(1, 1) = weight_forward;
  
  for (std::size_t i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0, pose_vertices_[i]);
    kinematics_edge->setVertex(1, pose_vertices_[i+1]);      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(params_);
    optimizer_->addEdge(kinematics_edge);
  }	 
}
*/
/*
// 2 Vertex Pose, All poses -1, 2 weights, 2D
void DiscreteTEBPlanner::AddEdgesKinematicsCarlike()
{
  const double weight_nh = params_.FollowPath.weights.weight_kinematics_nh;
  const double weight_turning_radius = params_.FollowPath.weights.weight_kinematics_turning_radius;
  if (weight_nh==0 && weight_turning_radius==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = weight_nh;
  information_kinematics(1, 1) = weight_turning_radius;
  
  for (std::size_t i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0,pose_vertices_[i]);
    kinematics_edge->setVertex(1,pose_vertices_[i+1]);      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(params_);
    optimizer_->addEdge(kinematics_edge);
  }  
}
*/
/*
// 3 Vertex Pose, 2nd to last-1, 1 weights, 1D
void DiscreteTEBPlanner::AddEdgesG3Continuity()
{
  const double weight = params_.FollowPath.weights.weight_g3_continuity;
  if (weight==0)
      return; // if weight equals zero skip adding edges!

  // We need at least 3 poses to define the first G3 continuity edge.
  if (teb_.sizePoses() < 3)
      return;

  Eigen::Matrix<double, 1, 1> information;
  information.fill(weight);
  // Iterate over all triplets of consecutive poses in the trajectory.
  // The loop runs from the second pose up to the second last pose.
  for (std::size_t i = 1; i+1 < teb_.sizePoses(); ++i)
  {
    EdgeG3Continuity* g3_edge = new EdgeG3Continuity();
    g3_edge->setVertex(0, pose_vertices_[i-1]);
    g3_edge->setVertex(1, pose_vertices_[i]);
    g3_edge->setVertex(2, pose_vertices_[i+1]);
    g3_edge->setInformation(information);
    g3_edge->setTebConfig(params_);
    optimizer_->addEdge(g3_edge);
  }
}
*/
/*
// 5 Vertex Pose, last 5 poses, 1 weights, 1D
void DiscreteTEBPlanner::AddEdgesSteeringAngleGoal()
{
  const double weight = params_.FollowPath.weights.weight_zero_steering_angle_goal;
  if (weight==0)
      return; // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();
  if (n < 5)
    return;
  // Only when path is near goal 
  if (teb_.backPose().toPoseMsg() != goal_)
    return;

  Eigen::Matrix<double, 1, 1> information_steering_goal;
  information_steering_goal(0, 0) = weight;
  EdgeSteeringAngleGoal* steering_goal_edge = new EdgeSteeringAngleGoal();
  steering_goal_edge->setVertex(0, pose_vertices_[n-5]);
  steering_goal_edge->setVertex(1, pose_vertices_[n-4]);
  steering_goal_edge->setVertex(2, pose_vertices_[n-3]);
  steering_goal_edge->setVertex(3, pose_vertices_[n-2]);
  steering_goal_edge->setVertex(4, pose_vertices_[n-1]);
  steering_goal_edge->setInformation(information_steering_goal);
  steering_goal_edge->setTebConfig(params_);
  optimizer_->addEdge(steering_goal_edge);
}
*/
/*
// 9 Vertex (5 Pose + 4 Time), last 5 poses, 1 weights, 1D
void DiscreteTEBPlanner::AddEdgesGoalAngularVelocityZero()
{
  const double weight = params_.FollowPath.weights.weight_goal_angular_vel_zero;
  if (weight==0)
      return; // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();
  if (n < 5)
    return;

  Eigen::Matrix<double, 1, 1> information;
  information(0, 0) = weight;
  EdgeGoalAngularVelocityZero* edge = new EdgeGoalAngularVelocityZero();
  int pose_end_idx = n - 1;
  int timediff_end_idx = teb_.sizeTimeDiffs() - 1;
  edge->setVertex(0, pose_vertices_[pose_end_idx - 4]);
  edge->setVertex(1, pose_vertices_[pose_end_idx - 3]);
  edge->setVertex(2, pose_vertices_[pose_end_idx - 2]);
  edge->setVertex(3, pose_vertices_[pose_end_idx - 1]);
  edge->setVertex(4, pose_vertices_[pose_end_idx]);
  edge->setVertex(5, timediff_vertices_[timediff_end_idx - 3]);
  edge->setVertex(6, timediff_vertices_[timediff_end_idx - 2]);
  edge->setVertex(7, timediff_vertices_[timediff_end_idx - 1]);
  edge->setVertex(8, timediff_vertices_[timediff_end_idx]);
  edge->setInformation(information);
  edge->setTebConfig(params_);
  optimizer_->addEdge(edge);
}
*/




