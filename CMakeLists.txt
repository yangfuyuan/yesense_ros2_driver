cmake_minimum_required(VERSION 3.5)
project(yesense_ros2_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial_driver REQUIRED)
set(dependencies rclcpp sensor_msgs serial_driver )

###########
## Build ##
###########

include_directories(
  include
  "${PROJECT_SOURCE_DIR}/include"
)

## Declare a C++ executable
add_executable(yesense_ros2_driver_node src/yesense_ros2_driver_node.cpp 
src/analysis_data.c src/yesense_ros2_driver.cpp)

## Specify libraries to link a library or executable target against

#############
## Install ##
#############

ament_export_include_directories(include)
ament_export_libraries(yesense_ros2_driver_node)
ament_target_dependencies(yesense_ros2_driver_node ${dependencies})
ament_package()

install(TARGETS yesense_ros2_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
