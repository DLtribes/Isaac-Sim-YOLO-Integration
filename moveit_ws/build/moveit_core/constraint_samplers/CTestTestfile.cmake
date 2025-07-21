# CMake generated Testfile for 
# Source directory: /root/ws_moveit/src/moveit2/moveit_core/constraint_samplers
# Build directory: /root/ws_moveit/build/moveit_core/constraint_samplers
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[test_constraint_samplers]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/root/ws_moveit/build/moveit_core/test_results/moveit_core/test_constraint_samplers.gtest.xml" "--package-name" "moveit_core" "--output-file" "/root/ws_moveit/build/moveit_core/ament_cmake_gmock/test_constraint_samplers.txt" "--command" "/root/ws_moveit/build/moveit_core/constraint_samplers/test_constraint_samplers" "--gtest_output=xml:/root/ws_moveit/build/moveit_core/test_results/moveit_core/test_constraint_samplers.gtest.xml")
set_tests_properties([=[test_constraint_samplers]=] PROPERTIES  LABELS "gmock" REQUIRED_FILES "/root/ws_moveit/build/moveit_core/constraint_samplers/test_constraint_samplers" TIMEOUT "60" WORKING_DIRECTORY "/root/ws_moveit/build/moveit_core/constraint_samplers" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;106;ament_add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;52;_ament_add_gmock;/root/ws_moveit/src/moveit2/moveit_core/constraint_samplers/CMakeLists.txt;40;ament_add_gmock;/root/ws_moveit/src/moveit2/moveit_core/constraint_samplers/CMakeLists.txt;0;")
subdirs("../gmock")
