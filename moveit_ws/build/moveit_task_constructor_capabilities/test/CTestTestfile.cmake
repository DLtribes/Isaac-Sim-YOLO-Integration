# CMake generated Testfile for 
# Source directory: /root/ws_moveit/src/moveit_task_constructor/capabilities/test
# Build directory: /root/ws_moveit/build/moveit_task_constructor_capabilities/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_execution "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/root/ws_moveit/build/moveit_task_constructor_capabilities/test_results/moveit_task_constructor_capabilities/test_execution.xunit.xml" "--package-name" "moveit_task_constructor_capabilities" "--output-file" "/root/ws_moveit/build/moveit_task_constructor_capabilities/launch_test/test_execution.txt" "--command" "/usr/bin/python3" "-m" "launch_testing.launch_test" "/root/ws_moveit/src/moveit_task_constructor/capabilities/test/test_execution.launch.py" "test_binary:=/root/ws_moveit/build/moveit_task_constructor_capabilities/test/test_execution" "--junit-xml=/root/ws_moveit/build/moveit_task_constructor_capabilities/test_results/moveit_task_constructor_capabilities/test_execution.xunit.xml" "--package-name=moveit_task_constructor_capabilities")
set_tests_properties(test_execution PROPERTIES  LABELS "launch_test" TIMEOUT "60" WORKING_DIRECTORY "/root/ws_moveit/build/moveit_task_constructor_capabilities/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/launch_testing_ament_cmake/cmake/add_launch_test.cmake;131;ament_add_test;/root/ws_moveit/src/moveit_task_constructor/capabilities/test/CMakeLists.txt;12;add_launch_test;/root/ws_moveit/src/moveit_task_constructor/capabilities/test/CMakeLists.txt;0;")
subdirs("../gtest")
