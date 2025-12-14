cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME rslcpp_time_delay_backend)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  ament_cmake
)
foreach(pkg_name ${DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

# ==============================================================================
# BUILD RSLCPP
# ==============================================================================

add_library(${PROJECT_NAME} SHARED
  src/callback_backend.cpp
  src/delay_backend.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)
ament_export_dependencies(${DEPENDENCIES}) # other dependencies so client does not need to call find_package on them
ament_package()


