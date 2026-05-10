## Requirements:
- Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- Docker
## Environment:
Uses Orca4 (https://github.com/clydemcqueen/orca4?tab=readme-ov-file) Dockerfile.
- Clone repository
- Build the Docker image:
  ```
  cd orca4/docker
  ./build.sh
  ```
- Start container:
  ```
  docker start orca4_sim
  ```
- Run container:
  ```
  docker exec -it orca4_sim /bin/bash
  ```
- Build the workspace:
  ```
  cd ~/colcon_ws
  colcon build
  ```
- Source workspace:
  ```
  source install/local_setup.bash
  ```
## Run Simulation
- Add `pipe.world` to `~/colcon/src/orca4/orca_description/worlds`
- Add `sim_pipe_launch.py` to `~/colcon/src/orca4/orca_bringup/launch`
- In the terminal run:
  ```
  ros2 launch orca_bringup sim_pipe_launch.py nav:=false rviz:=false slam:=false
  ```
## Run Controller
- Make ROS package in `~/colcon/src/orca4`
  ```
  ros2 pkg create --build-type ament_python orca_control --dependencies rclpy std_msgs
  ```
- Add `pc_controller.py` to `~/colcon/src/orca4/orca_control/orca_control`
- Rebuild
  ```
  cd ~/colcon/src
  colcon build
  source install/local_setup.bash
  ```
- Run
  ```
  ros2 run orca_control mpc_controller.py
  ```
  
