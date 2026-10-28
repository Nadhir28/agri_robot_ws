cmake_minimum_required(VERSION 3.8)
project(agri_ugv_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config worlds launch models
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/motion_test.py scripts/orchard_test.py scripts/sensor_test.py scripts/cloud_to_scan.py scripts/run_tests.sh scripts/gz_sim_wrapper.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
