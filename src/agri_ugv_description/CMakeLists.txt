cmake_minimum_required(VERSION 3.8)
project(agri_ugv_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config worlds launch models maps
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/motion_test.py scripts/orchard_test.py scripts/sensor_test.py scripts/cloud_to_scan.py scripts/rtk_gnss.py scripts/rtk_test.py scripts/gps_monitor.py scripts/sensor_covariance.py scripts/ekf_test.py scripts/ekf_monitor.py scripts/map_test.py scripts/run_tests.sh scripts/gz_sim_wrapper.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
