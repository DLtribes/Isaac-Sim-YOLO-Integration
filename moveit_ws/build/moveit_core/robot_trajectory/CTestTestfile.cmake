# CMake generated Testfile for 
# Source directory: /root/ws_moveit/src/moveit2/moveit_core/robot_trajectory
# Build directory: /root/ws_moveit/build/moveit_core/robot_trajectory
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[test_robot_trajectory]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/root/ws_moveit/build/moveit_core/test_results/moveit_core/test_robot_trajectory.gtest.xml" "--package-name" "moveit_core" "--output-file" "/root/ws_moveit/build/moveit_core/ament_cmake_gtest/test_robot_trajectory.txt" "--command" "/root/ws_moveit/build/moveit_core/robot_trajectory/test_robot_trajectory" "--gtest_output=xml:/root/ws_moveit/build/moveit_core/test_results/moveit_core/test_robot_trajectory.gtest.xml")
set_tests_properties([=[test_robot_trajectory]=] PROPERTIES  LABELS "gtest" REQUIRED_FILES "/root/ws_moveit/build/moveit_core/robot_trajectory/test_robot_trajectory" TIMEOUT "60" WORKING_DIRECTORY "/root/ws_moveit/build/moveit_core/robot_trajectory" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/root/ws_moveit/src/moveit2/moveit_core/robot_trajectory/CMakeLists.txt;19;ament_add_gtest;/root/ws_moveit/src/moveit2/moveit_core/robot_trajectory/CMakeLists.txt;0;")
