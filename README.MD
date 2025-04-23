# Component Localization
- gps & laser localization(using cyberrock-ivfc)

## Use
- load docker image
  
  docker load -i lidar_gimbal_loc.tar

- run docker container
  
  see run_docker.sh

- catkin_make

- mapping
  
  see mapping.sh

- localization
  
  see localization.sh

- lidar node for deskew
  
  rosrun lidar lidar_node

- gimbal control node
  
  rosrun lidar pelco_control pelco_control_node