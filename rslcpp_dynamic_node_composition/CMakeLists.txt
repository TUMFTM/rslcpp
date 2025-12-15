# Copyright 2025 Simon Sagmeister
cmake_minimum_required(VERSION 3.12)

# CONFIGURE PKG HERE
# ==========================================

set(PROJECT_NAME rslcpp_dynamic_node_composition)

set(LIB_DEPENDENCIES
  ament_cmake
  ament_index_cpp
  rclcpp
  rclcpp_components
)
# ============================================
# END OF CONFIGURATION

project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

foreach(pkg_name ${LIB_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

# section: BUILD LIBRARY TARGET TO IMPORT NODE CLASS WITHIN OTHER PKGS
# ====================================================================

add_library(${PROJECT_NAME} SHARED src/composition.cpp src/utils.cpp)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_target_dependencies(${PROJECT_NAME} ${LIB_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)
ament_export_dependencies(${LIB_DEPENDENCIES}) # other dependencies so client does not need to call find_package on them

# section: GENERAL
# ====================================================================
ament_package()