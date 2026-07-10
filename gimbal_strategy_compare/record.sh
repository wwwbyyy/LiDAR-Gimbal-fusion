rosbag record -O output.bag \
  /iekf3d/odometry \
  /gimbal_cmd \
  /pan /tilt \
  /local_map \
  /driver/hesai/pandar \
  /processed_cloud \
  /iekf3d/global_map \
  /iekf3d/registrationd_cloud