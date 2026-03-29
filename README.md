# UAV_hoist_Gazebo

An Gazebo simulation environment tailored for UAV (Unmanned Aerial Vehicle) lifting and hoisting operations.   
This project builds upon the robust foundation of the [PX4-ROS2-Gazebo Drone Simulation Template](https://github.com/SathanBERNARD/PX4-ROS2-Gazebo-Drone-Simulation-Template.git), incorporating significant enhancements for improved compatibility and functionality.

## Key Features & Enhancements

*   **PX4 v1.16.1 Compatibility:** The core simulation world (`test_world.sdf`) has been reconstructed following official PX4 templates to ensure seamless integration with PX4 Autopilot version 1.16.1.
*   **Resolved Compass Issue:** Addresses and resolves the QGroundControl (QGC) error `"Found 0 compass"`, ensuring proper sensor initialization and vehicle health checks.
*   **Specialized Simulation World:** Introduces a new, dedicated world featuring a `basketball_court.sdf` model, providing a structured environment for testing and demonstrating UAV hoisting scenarios.

## System Requirements

| Component | Version |
| :--- | :--- |
| Ubuntu | 22.04 |
| ROS 2 | Humble Hawksbill |
| PX4 Autopilot | 1.16.1 |
| px4_msgs & px4_ros_com | release/1.16 |
| Micro-XRCE-DDS-Agent | 3.0.1 |