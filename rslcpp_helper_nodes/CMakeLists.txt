cmake_minimum_required(VERSION 3.8)
project(rslcpp_helper_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(DEFINED ENV{ROS_DISTRO})
    string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO_UPPER)
    add_definitions(-DROS_DISTRO_${ROS_DISTRO_UPPER})
endif()

set(NODE_DEPENDENCIES
  rclcpp
  rclcpp_components
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  rslcpp
  rslcpp_dynamic_job
  diagnostic_msgs
)

find_package(ament_cmake)
foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

# section: BUILD COMPONENTS
# ====================================================================

set(COMPONENT_PATHS
  components/monitor.cpp
  components/player.cpp
  components/recorder.cpp
)

set(COMPONENT_NAMES
  ${PROJECT_NAME}::SimulationMonitor
  ${PROJECT_NAME}::BagPlayer
  ${PROJECT_NAME}::BagRecorder
)

set(EXECUTABLE_NAMES
  simulation_monitor_component
  bag_player_component
  bag_recorder_component
)

add_library(${PROJECT_NAME}_components SHARED ${COMPONENT_PATHS})
# Link the additional dependencies
ament_target_dependencies(${PROJECT_NAME}_components ${NODE_DEPENDENCIES})

target_include_directories(${PROJECT_NAME}_components
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

foreach(COMPONENT_NAME EXECUTABLE_NAME IN ZIP_LISTS COMPONENT_NAMES EXECUTABLE_NAMES)
  # Register component and executable for each node
  rclcpp_components_register_node(
    ${PROJECT_NAME}_components
      PLUGIN ${COMPONENT_NAME}
      EXECUTABLE ${EXECUTABLE_NAME}
  )
endforeach()

ament_export_targets(export_${PROJECT_NAME}_components)
ament_export_dependencies(${NODE_DEPENDENCIES})

install(TARGETS ${PROJECT_NAME}_components
        EXPORT export_${PROJECT_NAME}_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
