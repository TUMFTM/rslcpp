cmake_minimum_required(VERSION 3.8)
project(rslcpp_test)

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
  std_msgs
  std_srvs
  rslcpp
  rslcpp_helper_nodes
  rslcpp_time_delay_backend
  rslcpp_time_delay
)

find_package(ament_cmake)
foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()


# ==============================================================================
# BUILD EXECUATBLES
# ==============================================================================

set(EXECUTABLES
    time_delay
    communication_delay
    determinism
)

foreach(exec_name ${EXECUTABLES})
  # Create the executable
  add_executable(${exec_name} executables/${exec_name}.cpp)
  # Link against the lib
  target_include_directories(${exec_name}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
  ament_target_dependencies(${exec_name} ${NODE_DEPENDENCIES})
  # Install target
  install(
    TARGETS ${exec_name}  
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

ament_package()
