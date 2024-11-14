cmake_minimum_required(VERSION 3.5)
project(robot_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  xacro
  geometry_msgs
)
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  controller/speed_controller.cpp  
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(speed_controller_node controller/speed_controller.cpp)
ament_target_dependencies(speed_controller_node ${dependencies})
target_link_libraries(speed_controller_node ${PROJECT_NAME})

install(TARGETS
  speed_controller_node
  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch meshes urdf rviz 
  DESTINATION share/${PROJECT_NAME}
  
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # ament_add_pytest_test(ur_urdf_xacro test/test_ur_urdf_xacro.py)
  # ament_add_pytest_test(view_ur_launch test/test_view_ur_launch.py)
endif()
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
