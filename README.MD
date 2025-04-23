# Component Localization
- gps & laser localization(using cyberrock-ivfc)

## Use
- load docker image
  
  docker load -i lidar_gimbal_loc.tar

- run docker container
  
  see run_docker.sh

- build
  
  catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

- mapping
  
  see mapping.sh

- localization
  
  see localization.sh
  rviz -d iekf3d/configs/lidar3d.rviz

- lidar node for deskew
  
  rosrun lidar lidar_node

- gimbal control node
  
  rosrun lidar pelco_control pelco_control_node