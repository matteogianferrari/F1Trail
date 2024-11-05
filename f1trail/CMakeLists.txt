cmake_minimum_required(VERSION 3.5)
project(f1trail)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(zed_wrapper REQUIRED)
find_package(urg_node REQUIRED)
find_package(sl_stereo_proc REQUIRED)
find_package(aruco_loc REQUIRED)
find_package(clustering_proc REQUIRED)
find_package(tracking_proc REQUIRED)
find_package(longitudinal_pid REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
