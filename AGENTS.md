# AGENTS.md — nav2_teb_controller

## Overview

Nav2 controller plugin implementing TEB-K: Timed Elastic Band with polynomial (cubic Hermite) trajectory segments. Optimizes robot trajectories using sparse nonlinear least-squares (g2o) considering kinematics, dynamics, obstacle avoidance, and execution time.

Key dependencies: `libg2o`, `Eigen3`, `pluginlib`, `costmap_converter`, `ackermann_msgs`.

## Build & Test

```bash
make build       # colcon build --packages-select nav2_teb_controller
make test        # colcon test + test-result
make format      # clang-format --dry-run --Werror
make format-fix  # clang-format -i
make lint        # run-clang-tidy
make lint-fix    # clang-tidy --fix
```

Individual test:
```bash
colcon test --packages-select nav2_teb_controller --event-handlers console_direct+ --ctest-args -R test_edge_time_optimal
```

## Directory Layout

```
├── config/
│   ├── teb_controller_parameters.yaml   # generate_parameter_library schema
│   └── teb_controller_params.yaml       # Nav2 runtime config example
├── include/nav2_teb_controller/
│   ├── core/                            # Data structures
│   │   ├── timed_elastic_band.hpp       # TEB container (poses + dt)
│   │   ├── pose_se2.hpp                 # SE(2) pose
│   │   ├── footprint.hpp               # Robot footprint (circles/polygon)
│   │   └── teb_utils.hpp               # Utilities (init, resize, velocity cmd, feasibility)
│   ├── g2o_types/                       # 31 custom g2o edges + 2 vertices
│   │   ├── vertex_pose.h / vertex_timediff.h
│   │   ├── base_teb_edges.h             # Base edge templates
│   │   ├── penalties.h                  # Penalty functions
│   │   ├── edge_acceleration*.h, edge_jerk*.h, edge_snap.h
│   │   ├── edge_velocity*.h, edge_kinematic_*.h
│   │   ├── edge_costmap_obstacle.h, edge_esdf_obstacle.h
│   │   ├── edge_time_optimal.h, edge_shortest_path.h, edge_path_smoothness.h
│   │   ├── edge_steering_*.h, edge_goal_angular_velocity_zero.h
│   │   ├── edge_g3_continuity.h, edge_prefer_rotdir.h
│   │   └── edge_via_point.h
│   ├── homotopy/                        # Homotopy Class Planning
│   │   ├── homotopy_class_planner.hpp   # Multi-topology planner wrapper
│   │   ├── h_signature.hpp             # Complex-analysis homotopy signature
│   │   ├── teb_candidate.hpp           # One TEB + its homotopy class
│   │   ├── graph_search_interface.hpp  # Abstract graph search
│   │   ├── visibility_graph.hpp        # Visibility graph data structure
│   │   └── visibility_graph_search.hpp # Dijkstra + Yen's K-Shortest Paths
│   ├── obstacles/esdf.hpp              # Meijster EDT (O(n) distance transform)
│   ├── planner/
│   │   ├── planner_interface.hpp        # PlannerBase + PlannerInterface<TEB> ABCs
│   │   └── optimal_planner.hpp         # DiscreteTEBPlanner (g2o graph builder)
│   ├── visualization/teb_visualizer.hpp # 7 RViz publishers
│   └── math_utils.hpp                   # average_angle, fast_sigmoid
├── src/
│   ├── teb_controller.cpp              # Main plugin: TEBController
│   ├── core/teb_utils.cpp              # Full impl of utilities
│   ├── planner/optimal_planner.cpp     # Graph building + optimization
│   ├── homotopy/homotopy_class_planner.cpp  # Stub
│   ├── homotopy/visibility_graph_search.cpp # Stub
│   └── obstacles/esdf.cpp              # Meijster EDT algorithm
├── test/unit/g2o_types/                # 3 gtest files for g2o edges
├── scripts/footprint_polygon_to_circles.py
├── launch/nav2_teb_controller_launch.py
└── doc/architecture.md
```

## Architecture

### Data Flow

```
Global Plan (nav_msgs/Path)
  → pruneGlobalPlan + transformAndTrimPlan (lookahead window)
  → Optional: HomotopyClassPlanner
      → VisibilityGraphSearch (Dijkstra + Yen's K-Shortest + H-Signature filter)
      → DiscreteTEBPlanner per candidate
      → Select best feasible candidate
  → DiscreteTEBPlanner.plan()
      → initFromPath / updateAndPrune (warm-start)
      → autoResize (insert/delete poses by dt/angle)
      → buildGraph (g2o: vertices + edges for all constraints)
      → optimizeGraph (Gauss-Newton / Levenberg-Marquardt)
      → writeBackOptimizedValues
  → checkFeasibility (ESDF + footprint collision check)
  → getVelocityCommand (lookahead velocity extraction)
  → saturateVelocity + saturateSteeringAngle
  → TwistStamped cmd_vel
```

### g2o Graph Structure

- **Vertices**: `VertexPose` (SE2, optimized) per pose, `VertexTimeDiff` (dt, optimized) per segment. Start/goal poses fixed.
- **Edges** grouped into 3 categories:
  1. **Generic** (via `addEdgesGeneric` template): Velocity, TimeOptimal, ShortestPath, PathSmoothness, Snap, G3Continuity, SteeringAngleGoal, GoalAngularVelocityZero, KinematicsDiffDrive, KinematicsCarlike
  2. **Measurement** (manual setup): Acceleration (+Start/+Goal), Jerk (+Start/+Goal), SteeringRate (+Start/+Goal), StartSteeringAngle
  3. **Obstacle**: ESDF-based (per-pose, footprint-aware with robust kernel)
- Solvers: eigen (default), cholmod, csparse, dense
- Algorithms: gauss_newton (default), levenberg_marquardt

### ESDF (ObstacleMap2D)

- Meijster algorithm: O(n) EDT on binary occupancy grid
- Bilinear interpolation for C¹-continuous query
- Analytical gradient (normalized, Eikonal constraint)
- Used for both collision checking and gradient-based obstacle edge

## Status

### ✅ Implemented
- TEBController plugin (configure, activate, deactivate, cleanup, setPlan, computeVelocityCommands, setSpeedLimit)
- Costmap converter integration
- ESDF (Meijster EDT) with query/gradient
- Footprint (circle model, polygon→circles, ESDF-based collision check)
- initFromPath, autoResize, updateAndPrune, extractVelocity, getVelocityCommand
- saturateVelocity, saturateSteeringAngle, convertAckermannToTwist, convertTwistToAckermann
- computeCurvature, checkFeasibility
- pruneGlobalPlan, transformAndTrimPlan
- All 31 g2o edge types + 2 vertex types
- Generic edge adder (`addEdgesGeneric` template)
- Graph building with all constraint edges
- Optimization (inner/outer loop, weight adaptation, divergence detection)
- Stepwise optimization: 3-phase pipeline (obstacles+G3+kinematics → kinodynamics → full)
- Cost computation with per-edge-type breakdown
- TEBVisualizer (7 topics: local_plan, lookahead, obstacles, curvature radii, footprint)
- 3 unit tests (time_optimal, path_smoothness, shortest_path)
- Parameter library via `generate_parameter_library`
- Homotopy Class Planning interfaces (stubs)
- Visibility Graph data structure (stub search)

### 🚧 TODO / Stubs
- `HomotopyClassPlanner::plan()` — actual multi-candidate orchestration
- `VisibilityGraphSearch::search()` — Dijkstra + Yen's K-Shortest + H-Signature
- `VisibilityGraph::build()` — keypoint extraction + visibility checks
- `HSignature::compute()` — complex integral along path
- ESDF-aware autoResize (nudge midpoint away from obstacles)
- `EdgeViaPoints` — via-point constraint
- `AddEdgesPreferRotDir` — preferred rotation direction
- `AddEdgesObstaclesLegacy` — legacy costmap obstacle association
- Recovery behaviors module
- Integration tests (test/integration/ is empty)
- `EdgeJerk` (the old one, `edge_jerk.h`) — only `edge_jerk_new.h` is used
- `edge_prefer_rotdir.h` — not wired up

## Conventions

- C++20, Google style (`.clang-format`), clang-tidy with relaxed rules
- Namespace: `nav2_teb_controller`
- Headers: `#pragma once` (new code), `#ifndef` guards (legacy g2o types)
- `class` → CamelCase, `function` → camelCase, `variable`/`member` → snake_case
- `_` suffix for class members (private)
- `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` on classes with Eigen members
- ROS2 lifecycle node pattern for plugin
- Parameters via `generate_parameter_library` (code-generated from YAML schema)
- g2o edges own their vertex connections (raw `new`/`delete` by design, per clang-tidy suppression)
