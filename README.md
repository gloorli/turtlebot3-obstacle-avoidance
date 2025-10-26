# 🐢 TurtleBot3 Obstacle Avoidance (ROS 2 + Gazebo + Docker)

Autonomous obstacle avoidance for the TurtleBot3 Burger using ROS 2 (Humble) and Gazebo, fully containerized with Docker.

This README covers the workflow for running and developing the turtlebot3_autonomy package in a containerized ROS 2 environment.

---

## Prerequisites (Host)

1. Install Docker and the compose plugin:
```bash
sudo apt update
sudo apt install -y docker.io docker-compose-plugin
sudo systemctl enable --now docker
sudo usermod -aG docker $USER
newgrp docker
```

2. (Optional) Enable X11 access for Gazebo GUI:
```bash
sudo apt install -y x11-xserver-utils
xhost +local:root
```

3. Clone this repository into your development folder:
```bash
cd ~/dev
git clone <repo-url>
cd turtlebot3-obstacle-avoidance
```

---

## Running the simulation (host)

1. Build and start the container:
```bash
docker compose build
docker compose up -d
```

2. Verify the container is running:
```bash
docker ps
```

3. Open a shell in the running container:
```bash
docker exec -it turtlebot3_obstacle_avoidance bash
```

4. Source (ROS2 / workspace) and build packages:
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

5. Launch Gazebo with the TurtleBot3 world:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
(You should see Gazebo open with a TurtleBot3 Burger in a small world.)

---

## Teleoperation (second terminal)

Open a second shell:
```bash
docker exec -it turtlebot3_obstacle_avoidance bash
```

Then source again:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Run a teleop node:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
Use keyboard teleop to move the robot manually.

---

## Autonomous obstacle avoidance (third terminal)

Open a third shell:
```bash
docker exec -it turtlebot3_obstacle_avoidance bash
```

Then source again:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Run obstacle avoidance node:
```bash
ros2 run turtlebot3_autonomy obstacle_avoidance
```
The robot should drive autonomously and avoid obstacles.

---

## Development notes

- After modifying source code, rebuild inside the container:
```bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

- If Gazebo GUI does not appear, ensure X11 forwarding is permitted (see Prerequisites). Alternatively, run Gazebo headless.
