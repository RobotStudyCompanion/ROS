cmake_minimum_required(VERSION 3.8)
project(rviz_camera_stream)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(example_interfaces REQUIRED)


# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

# Enable Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Avoid defining "emit", "slots", etc., for Qt signals
add_definitions(-DQT_NO_KEYWORDS)

# Include directories
include_directories(
  include
  ${example_interfaces_INCLUDE_DIRS}
)

# Source files
set(SOURCES
  src/camera_display.cpp
)

set(HEADERS
  include/${PROJECT_NAME}/camera_display.h
)

# Wrap Qt MOC files
qt5_wrap_cpp(MOC_FILES ${HEADERS})

# Create the library
add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
  ${MOC_FILES}
)

# Link against libraries
target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    rviz_common::rviz_common
    rviz_rendering::rviz_rendering
    rviz_ogre_vendor::OgreMain
    rclcpp::rclcpp
    image_transport::image_transport
    ${example_interfaces_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${std_srvs_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${example_interfaces_INCLUDE_DIRS}
)

# Export the plugin
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  plugin_description.xml
  DESTINATION share/ament_index/resource_index/rviz_plugin_description
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Ament package configuration
ament_export_dependencies(
  rclcpp
  image_transport
  sensor_msgs
  example_interfaces
  std_srvs
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
)

ament_package()

