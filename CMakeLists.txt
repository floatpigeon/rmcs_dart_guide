cmake_minimum_required(VERSION 3.8)
project(rmcs_dart_guide)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-packed-bitfield-compat")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (CONFIGURE_DIR_PATH ${PROJECT_SOURCE_DIR}/src)

# find dependencies
find_package(OpenCV 4.5 REQUIRED)

# Include project source directory
include_directories(${PROJECT_SOURCE_DIR}/src ${CONFIGURE_DIR_PATH})

file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
        ${PROJECT_SOURCE_DIR}/src/*.cpp
        ${PROJECT_SOURCE_DIR}/src/*.cc)

find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

ament_auto_add_library (
  ${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} -lpthread)
pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)


# Add save_pointcloud_node
add_executable(pointcloud_save_node src/pointcloud_data/lidar_data_save_node.cpp)

ament_target_dependencies(pointcloud_save_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_include_directories(pointcloud_save_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(pointcloud_save_node
  ${PCL_LIBRARIES}
)

install(TARGETS pointcloud_save_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
