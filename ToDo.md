# Project Improvements To-Do List

- [x] **Memory Safety:** Evaluate the deletion of copy constructors in `TimedElasticBand`. Consider implementing efficient deep copies or ensuring move-only semantics are handled safely throughout the `HomotopyClassPlanner`.
- [ ] **Code Duplication:** Refactor G2O edges (Acceleration, Velocity, Jerk) into template-based hierarchies to share logic between holonomic and non-holonomic implementations.
- [ ] **Interface Compliance:** Audit G2O edge classes (specifically `EdgeShortestPath` and `EdgePathSmoothness`) to ensure `computeError` and other virtual methods use `override` and `const` correctly.
- [ ] **Footprint Abstraction:** Unify obstacle edges to use the `Footprint` class instead of raw `std::vector<geometry_msgs::msg::Point>` to ensure consistent collision checking logic.
