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
- Add `orca_control` package to `~/colcon/src/orca4`
- Rebuild
  ```
  cd ~/colcon/src
  colcon build
  source install/local_setup.bash
  ```
- Run
  ```
  ros2 run orca_control nmpc_controller
  ```
  
## Publish waypoint example
  ```
  ros2 topic pub /mpc/eta_ref std_msgs/msg/Float64MultiArray \
  "{data: [4.0, 4.0, -4.0, 0.0, 0.0, 0.0]}"
  ```

## Example video
https://github.com/user-attachments/assets/57305547-79df-403f-b263-314aa6975227

<img width="1141" height="403" alt="traj_plt" src="https://github.com/user-attachments/assets/9848a417-5fab-4dd3-a8f1-5d6b2286dfea" />
<img width="500" height="357" alt="err_plt" src="https://github.com/user-attachments/assets/7fdc8fb5-eff6-4589-afb3-df7b10b38888" />
<img width="500" height="357" alt="control_input" src="https://github.com/user-attachments/assets/a794dce4-87a3-4c16-8150-06611fad1598" />



