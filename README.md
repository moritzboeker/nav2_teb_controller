# nav2_teb_controller

A modern C++ reimplementation of the **Timed Elastic Band (TEB)** local planner, redesigned for the [Nav2](https://nav2.ros.org/) navigation stack on **ROS2**.

This package is based on the original [`teb_local_planner`](https://github.com/rst-tu-dortmund/teb_local_planner) by [RST – TU Dortmund](https://github.com/rst-tu-dortmund), adapted and extended for modern ROS2 standards, improved optimization backends, and enhanced kinodynamic constraint handling.

***

## ✨ Features

- Full ROS2 / Nav2 integration as a pluginlib-based local planner controller
- Trajectory optimization using the **g2o** sparse nonlinear least-squares framework
- Support for differential drive, bicycle, and tricycle kinematic models
- Dynamic obstacle avoidance with configurable inflation and costmap integration
- Improved code structure following modern C++17/20 practices
- Drop-in replacement for the classic `teb_local_planner` in ROS2 environments

***

## 📦 Dependencies

- ROS2 (Humble or newer)
- Nav2 (`nav2_core`, `nav2_costmap_2d`, `nav2_util`)
- `g2o` — graph optimization framework
- `Eigen3`
- `pluginlib`

***

## 🚀 Build & Install

```bash
# Clone into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/danielyousef/nav2_teb_controller.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select nav2_teb_controller --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source
source install/setup.bash
```

***

## ⚙️ Configuration

Add the controller plugin to your Nav2 params file:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_teb_controller::TEBController"
```

For a full list of configurable parameters, refer to the [parameter documentation](docs/parameters.md).

***

## 🎥 Demo Videos

| Video | Link |
|-------|------|
| Demo 1 | [▶ Watch on YouTube](https://www.youtube.com/watch?v=3201fSL2xvI) |
| Demo 2 | [▶ Watch on YouTube](https://www.youtube.com/watch?v=a3creSJZxis) |
| Demo 3 | [▶ Watch on YouTube](https://www.youtube.com/watch?v=BqfFwOXZO0k) |

***

## 📚 Original Work & References

This package is derived from and inspired by:

- **teb_local_planner** — [github.com/rst-tu-dortmund/teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)
  - C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann, T. Bertram: *Trajectory modification considering dynamic constraints of autonomous robots*, ROBOTIK 2012.
  - C. Rösmann, F. Hoffmann, T. Bertram: *Integrated online trajectory planning and optimization in distinctive topologies*, Robotics and Autonomous Systems, 2017.

***

## 📄 License

This project is licensed under the **BSD 3-Clause License** — see [LICENSE](LICENSE) for details.

The original `teb_local_planner` is also licensed under BSD 3-Clause.

***

## 🙋 Author

**Daniel Yousef**
[GitHub](https://github.com/danielyousef)