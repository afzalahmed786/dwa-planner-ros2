cat > README.md << 'EOF'
# Custom DWA Local Planner for ROS2 Foxy

A custom implementation of the Dynamic Window Approach (DWA) local planner for TurtleBot3 navigation in ROS2 Foxy.

## Overview

This package implements a DWA-based local planner from scratch (without using nav2_dwb_controller) for autonomous navigation of a TurtleBot3 robot in Gazebo simulation.

## Features

- **Dynamic velocity sampling** within robot constraints
- **Multi-objective cost function** considering:
  - Distance to goal
  - Heading alignment
  - Velocity optimization
  - Obstacle avoidance
- **Real-time trajectory visualization** in RViz
- **Emergency stop** functionality for safety
- **Configurable parameters** for different scenarios

## Prerequisites

- Ubuntu 20.04
- ROS2 Foxy
- TurtleBot3 packages
- Gazebo 11
- Python 3.8+

## Installation

### 1. Install Dependencies
```bash
# Install ROS2 Foxy (if not already installed)
sudo apt update
sudo apt install ros-foxy-desktop

# Install TurtleBot3 packages
sudo apt install ros-foxy-turtlebot3*
sudo apt install ros-foxy-gazebo-ros-pkgs
```

### 2. Clone and Build
```bash
# Create workspace
mkdir -p ~/dwa_ws/src
cd ~/dwa_ws/src

# Clone this repository
git clone https://github.com/YOUR_USERNAME/custom-dwa-planner-ros2.git custom_dwa_planner

# Build the package
cd ~/dwa_ws
colcon build --packages-select custom_dwa_planner
source install/setup.bash
```

## Usage

### Terminal 1: Launch Gazebo Simulation
```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/foxy/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2: Run DWA Planner
```bash
source ~/dwa_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run custom_dwa_planner dwa_planner --ros-args -p use_sim_time:=true
```

### Terminal 3: Set Navigation Goal
```bash
source ~/dwa_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run custom_dwa_planner goal_publisher --ros-args -p use_sim_time:=true -p goal_x:=2.0 -p goal_y:=1.0
```

### Terminal 4 (Optional): Visualize in RViz
```bash
source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=burger
rviz2
```

**In RViz:**
1. Set **Fixed Frame** to `odom`
2. Add **MarkerArray** → Topic: `/dwa_trajectories`
3. Add **LaserScan** → Topic: `/scan`
4. Add **RobotModel**

## Architecture

### Nodes

- **dwa_planner**: Main DWA local planner node
- **goal_publisher**: Simple goal publisher for testing

### Topics

**Subscribed:**
- `/odom` (nav_msgs/Odometry): Robot odometry
- `/scan` (sensor_msgs/LaserScan): Laser scan data
- `/goal_pose` (geometry_msgs/PoseStamped): Goal position

**Published:**
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/dwa_trajectories` (visualization_msgs/MarkerArray): Trajectory visualization

## Configuration Parameters

Edit `DWAConfig` class in `dwa_planner_node.py` to adjust:
```python
max_speed = 0.12           # Maximum linear velocity (m/s)
max_yaw_rate = 1.0         # Maximum angular velocity (rad/s)
max_accel = 0.3            # Maximum linear acceleration (m/s²)
predict_time = 1.5         # Prediction horizon (s)
robot_radius = 0.25        # Robot radius for collision checking (m)
goal_tolerance = 0.3       # Goal reached threshold (m)

# Cost function weights
heading_cost_gain = 0.5
distance_cost_gain = 1.0
velocity_cost_gain = 0.1
obstacle_cost_gain = 3.0
```

## Algorithm

1. **Calculate Dynamic Window**: Determine feasible velocity space based on current velocity and robot constraints
2. **Sample Velocities**: Generate velocity pairs (v, ω) within the dynamic window
3. **Predict Trajectories**: Simulate robot motion for each velocity sample
4. **Evaluate Costs**: Calculate multi-objective cost for each trajectory
5. **Select Best Command**: Choose velocity with lowest total cost
6. **Safety Check**: Emergency stop if obstacles are too close

## Project Structure
```
custom_dwa_planner/
├── custom_dwa_planner/
│   ├── __init__.py
│   ├── dwa_planner_node.py      # Main DWA planner implementation
│   └── goal_publisher.py         # Goal publisher utility
├── launch/
│   └── dwa_planner.launch.py    # Launch file
├── resource/
│   └── custom_dwa_planner
├── package.xml
├── setup.py
└── README.md
```

## Troubleshooting

### Robot Tips Over
- Reduce `max_speed`, `max_accel`, and `max_yaw_rate` in `DWAConfig`
- Increase `max_delta_yaw_rate` for smoother turns

### Robot Spins in Place
- Increase `heading_cost_gain`
- Decrease `max_yaw_rate`
- Adjust `velocity_cost_gain`

### Robot Collides with Obstacles
- Increase `obstacle_cost_gain`
- Increase `robot_radius` for larger safety margin
- Increase `min_obstacle_dist`

### No Movement
- Check if goal is published: `ros2 topic echo /goal_pose`
- Check if odometry is received: `ros2 topic echo /odom`
- Verify simulation time: `ros2 param get /dwa_planner use_sim_time`

## Demo

[Add screenshots or GIFs of your robot navigating here]

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the Apache License 2.0.

## Author

Your Name

## Acknowledgments

- Based on the Dynamic Window Approach by Fox et al. (1997)
- TurtleBot3 simulation environment by ROBOTIS
- ROS2 Foxy framework
EOF
