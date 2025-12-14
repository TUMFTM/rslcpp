cmake_minimum_required(VERSION 3.8)
project(rslcpp_time_delay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(DEFINED ENV{ROS_DISTRO})
    string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO_UPPER)
    add_definitions(-DROS_DISTRO_${ROS_DISTRO_UPPER})
endif()

set(NODE_DEPENDENCIES
  ament_index_cpp 
  rclcpp
  rclcpp_components
  rslcpp
  rslcpp_time_delay_backend
)

find_package(ament_cmake)
foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

# section: BUILD COMPONENTS
# ====================================================================

set(COMPONENT_PATHS
  components/time_delay_loader.cpp
)

set(COMPONENT_NAMES
  ${PROJECT_NAME}::TimeDelayLoader
)

set(EXECUTABLE_NAMES
  time_delay_loader_component
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

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
