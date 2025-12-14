cmake_minimum_required(VERSION 3.8)
project(rslcpp_dynamic_job)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(DEFINED ENV{ROS_DISTRO})
    string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO_UPPER)
    add_definitions(-DROS_DISTRO_${ROS_DISTRO_UPPER})
endif()

set(LIB_DEPENDENCIES
  rclcpp
  rclcpp_components
  rslcpp
  rslcpp_dynamic_node_composition
)

find_package(ament_cmake)
foreach(pkg_name ${LIB_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()


# section : Build library target
add_library(${PROJECT_NAME} SHARED src/dynamic_job.cpp src/backend.cpp)
ament_target_dependencies(${PROJECT_NAME} ${LIB_DEPENDENCIES})
target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
# ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${LIB_DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# section: Build executables
# ===================================================
set(exec_name dynamic_job)
add_executable(${exec_name} executables/dynamic_job.cpp)
  # Link against the lib
  target_include_directories(${exec_name}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${exec_name} ${PROJECT_NAME})
ament_target_dependencies(${exec_name} ${LIB_DEPENDENCIES})
  # Install target
install(
    TARGETS ${exec_name}  
    DESTINATION lib/${PROJECT_NAME}
  )


ament_export_dependencies(${LIB_DEPENDENCIES})
ament_package()
