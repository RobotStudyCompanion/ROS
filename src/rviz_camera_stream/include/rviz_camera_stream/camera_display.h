#ifndef RVIZ_CAMERA_STREAM_CAMERA_DISPLAY_H
#define RVIZ_CAMERA_STREAM_CAMERA_DISPLAY_H

#include <memory>
#include <mutex>
#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#include <OgreSharedPtr.h>
#include <OgreTexture.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <example_interfaces/srv/trigger.hpp> // Corrected to use example_interfaces

#include "rviz_common/display.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/display_group_visibility_property.hpp"
#include "rviz_common/display_context.hpp" // Ensure full definition of DisplayContext is included

namespace rviz
{

class CameraPub : public rviz_common::Display
{
  Q_OBJECT
public:
  CameraPub();
  ~CameraPub() override;

  void onInitialize() override;
  void reset() override;

private:
  void subscribe();
  void unsubscribe();

  // ROS 2 Node
  rclcpp::Node::SharedPtr node_;

  // ROS 2 Interfaces
  rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr trigger_service_; // Updated
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  sensor_msgs::msg::CameraInfo::SharedPtr current_caminfo_;

  // RViz Properties
  rviz_common::properties::RosTopicProperty* topic_property_; // Added declaration
  std::mutex caminfo_mutex_;
};

}  // namespace rviz

#endif  // RVIZ_CAMERA_STREAM_CAMERA_DISPLAY_H

