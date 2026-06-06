# Nav2 TEB Controller Architecture

This repository implements the Timed Elastic Band (TEB) local planner for the Nav2 (ROS 2) navigation stack. It optimizes robot trajectories by considering kinematic constraints, obstacle avoidance, and execution time.

## Core Components

### 1. Timed Elastic Band (TEB)
- **`TimedElasticBand`**: The central data structure (in `timed_elastic_band.hpp`) storing a sequence of SE(2) poses and the time intervals ($\Delta t$) between them.
- **`PoseSE2`**: A numerically stable representation of robot position and orientation ($x, y, \theta$).

### 2. Optimization Engine (g2o)
The planner formulates trajectory optimization as a hyper-graph problem using the **g2o** library.
- **Vertices**: `VertexPose` and `VertexTimediff`.
- **Edges (Constraints)**: Located in `include/nav2_teb_controller/g2o_types/`, including:
    - Kinematics (Differential drive, Car-like).
    - Dynamics (Acceleration, Jerk, Velocity limits).
    - Obstacle avoidance (Costmap-based or ESDF-based).
    - Path properties (Smoothness, Shortest path, Time optimality).

### 3. Obstacles & Footprint
- **`Footprint`**: Manages robot shapes using either circular approximations or polygons.
- **`ObstacleMap2D` (ESDF)**: Provides a Euclidean Distance Field for fast, gradient-based obstacle distance queries.
- **`footprint_polygon_to_circles.py`**: A utility script to decompose complex polygons into circular sets for efficient collision checking.

### 4. Homotopy Class Planning (HCP)
To avoid local minima, the planner can search for multiple distinct paths in different homotopy classes.
- **`HomotopyClassPlanner`**: Manages multiple TEB candidates.
- **`VisibilityGraphSearch`**: Builds a visibility graph to find initial paths in unique homotopy classes.
- **`HSignature`**: A complex-analysis based signature used to uniquely identify and filter homotopy classes.

### 5. Planner Interface
- **`DiscreteTEBPlanner`**: The primary implementation of the `PlannerInterface`. It coordinates the g2o optimizer, obstacle updates, and trajectory generation.
- **`TEBVisualizer`**: Handles ROS 2 visualization markers for the planned paths and robot footprints in RViz.

## Data Flow
1. **Input**: Global path (from a Nav2 planner) and current robot state (velocity/pose).
2. **Path Search**: (Optional) `VisibilityGraphSearch` finds alternative homotopy paths.
3. **Initialization**: The `TimedElasticBand` is initialized from the global path.
4. **Optimization**: `DiscreteTEBPlanner` builds the g2o graph and runs the optimizer to deform the "elastic band" away from obstacles while respecting limits.
5. **Output**: A velocity command or a local `nav_msgs/Path` for execution.
