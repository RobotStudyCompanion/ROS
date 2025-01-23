#include "rviz_camera_stream/camera_display.h"
#include <pluginlib/class_list_macros.hpp>
#include <QString>

namespace rviz
{

CameraPub::CameraPub()
{
  topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Image Topic", "",
      QString::fromStdString("sensor_msgs/msg/Image"),
      "Image topic to publish to.",
      this);
}

CameraPub::~CameraPub() = default;

void CameraPub::onInitialize()
{
  node_ = this->context_->getRosNodeAbstraction().lock()->get_raw_node();
  image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("image", 10);
  trigger_service_ = node_->create_service<example_interfaces::srv::Trigger>(
      "camera_trigger", [this](const std::shared_ptr<example_interfaces::srv::Trigger::Request>,
                               std::shared_ptr<example_interfaces::srv::Trigger::Response> response) {
        response->success = true;
        response->message = "Triggered successfully!";
      });

  subscribe();
}

void CameraPub::subscribe()
{
  if (!isEnabled())
    return;

  caminfo_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", 10,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(caminfo_mutex_);
        current_caminfo_ = msg;
      });
}

void CameraPub::unsubscribe()
{
  caminfo_sub_.reset();
}

void CameraPub::reset()
{
  unsubscribe();
  Display::reset();
}

}  // namespace rviz

PLUGINLIB_EXPORT_CLASS(rviz::CameraPub, rviz_common::Display)
#include "camera_display.moc"

