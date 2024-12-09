cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)



rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
    DEPENDENCIES
    "std_msgs"
    "geometry_msgs"
	"nav_msgs"
)
# This is the method for same package interface for previous ROS2 distro
# rosidl_get_typesupport_target(cpp_typesupport_target
#    ${PROJECT_NAME} rosidl_typesupport_cpp)

# This is the method for same package interface for ROS2 galactic
function(rosidl_get_typesupport_target var generate_interfaces_target typesupport_name)
  if(NOT TARGET ${generate_interfaces_target})
    message(FATAL_ERROR
      "${generate_interfaces_target} is not a CMake target. Maybe rosidl_generate_interfaces was given a different target name?")
  endif()

  set(output_target "${generate_interfaces_target}__${typesupport_name}")

  if(NOT TARGET ${output_target})
    # CMake if() evaluates strings ending in `-NOTFOUND` as false
    set(output_target "${output_target}-NOTFOUND")
  endif()

  set("${var}" "${output_target}" PARENT_SCOPE)
endfunction()


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# tortoisebot_action_server lib
set(HEADER_FILES include/tortoisebot_action_server.h)
add_library(tortoisebot_action_server_lib src/tortoisebot_waypoints/tortoisebot_action_server_libonly.cpp ${HEADER_FILES} )
ament_target_dependencies(tortoisebot_action_server_lib rclcpp rclcpp_action geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tortoisebot_action_server_lib "${cpp_typesupport_target}")
ament_export_targets(tortoisebot_action_server_lib HAS_LIBRARY_TARGET)

# tortoisebot_action_server lib to executable
add_executable(tortoisebot_waypoint_action_server_libexec_node src/tortoisebot_waypoints/tortoisebot_action_server_libmain.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tortoisebot_waypoint_action_server_libexec_node tortoisebot_action_server_lib "${cpp_typesupport_target}")
ament_target_dependencies(tortoisebot_waypoint_action_server_libexec_node rclcpp rclcpp_action geometry_msgs)

# tortoisebot_action_server direct executable
add_executable(tortoisebot_waypoint_action_server_node src/tortoisebot_waypoints/tortoisebot_action_server.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tortoisebot_waypoint_action_server_node "${cpp_typesupport_target}")
ament_target_dependencies(tortoisebot_waypoint_action_server_node rclcpp rclcpp_action geometry_msgs)

#-----------------

# tortoisebot_action_client lib
set(HEADER_FILES include/tortoisebot_action_client.h)
add_library(tortoisebot_action_client_lib src/tortoisebot_waypoints/tortoisebot_action_client_libonly.cpp ${HEADER_FILES} )
ament_target_dependencies(tortoisebot_action_client_lib rclcpp rclcpp_action geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tortoisebot_action_client_lib "${cpp_typesupport_target}")
ament_export_targets(tortoisebot_action_client_lib HAS_LIBRARY_TARGET)

# tortoisebot_action_client lib to executable
add_executable(tortoisebot_waypoint_action_client_libexec_node src/tortoisebot_waypoints/tortoisebot_action_client_libmain.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tortoisebot_waypoint_action_client_libexec_node tortoisebot_action_client_lib "${cpp_typesupport_target}")
ament_target_dependencies(tortoisebot_waypoint_action_client_libexec_node rclcpp rclcpp_action geometry_msgs)

# tortoisebot_action_client direct executbale
add_executable(tortoisebot_waypoint_action_client_node src/tortoisebot_waypoints/tortoisebot_action_client.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tortoisebot_waypoint_action_client_node "${cpp_typesupport_target}")
ament_target_dependencies(tortoisebot_waypoint_action_client_node rclcpp rclcpp_action geometry_msgs)



install(TARGETS
   tortoisebot_waypoint_action_server_node
   tortoisebot_waypoint_action_client_node
   tortoisebot_waypoint_action_server_libexec_node
   tortoisebot_waypoint_action_client_libexec_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# install(
#   DIRECTORY include/
#   DESTINATION include
# )


install(
  TARGETS tortoisebot_action_server_lib 
  EXPORT tortoisebot_action_server_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS tortoisebot_action_client_lib
  EXPORT tortoisebot_action_client_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
      test/main.cpp
      test/tortoisebot_action_server_test.cpp
      )
  # Add a gtest executables
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_test tortoisebot_action_client_lib tortoisebot_action_server_lib)

  install(TARGETS
        ${PROJECT_NAME}_test
        # Install the generated test executable file to the path after DESTINATION
        DESTINATION lib/${PROJECT_NAME})
  # uncomment to enforce lintners and static code analyzers defined in ament_lint_common package        
  # ament_lint_auto_find_test_dependencies()
endif()


ament_package()
