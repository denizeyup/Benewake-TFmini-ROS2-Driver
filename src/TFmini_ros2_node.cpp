#include "tfmini_ros2/TFmini.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tfmini_ros_node");

  std::string id = "TFmini"; // TF Frame id
  std::string topic_name = "range"; // ROS Topic name
  std::string portName;
  int baud_rate;
  benewake::TFmini *tfmini_obj;

  node->declare_parameter("serial_port", "/dev/serial/your/port");
  node->declare_parameter("baud_rate", 115200);
  node->get_parameter("serial_port", portName);
  node->get_parameter("baud_rate", baud_rate);

  // init TFmini variable & ros publisher
  tfmini_obj = new benewake::TFmini(portName, baud_rate); // create TFmini object
  auto pub_range = node->create_publisher<sensor_msgs::msg::Range>(topic_name, 10);

  // sensor msgs init
  sensor_msgs::msg::Range TFmini_range;
  TFmini_range.radiation_type = sensor_msgs::msg::Range::INFRARED;
  TFmini_range.field_of_view = 0.04;
  TFmini_range.min_range = 0.3;
  TFmini_range.max_range = 12;
  TFmini_range.header.frame_id = id;
  float dist = 0;

  RCLCPP_INFO(node->get_logger(), "Start processing TFmini...");

  rclcpp::Rate loop_rate(10); // 10 Hz döngü hızı

  while(rclcpp::ok()){
    dist = tfmini_obj->getDist();
    if(dist > 0 && dist < TFmini_range.max_range) {
      TFmini_range.range = dist;
      RCLCPP_INFO(node->get_logger(), "Publishing valid distance: %f", dist);
    }
    else if(dist == 0.0) {
      TFmini_range.range = TFmini_range.max_range;
      RCLCPP_INFO(node->get_logger(), "Distance out of range, using max range value.");
    }
    TFmini_range.header.stamp = node->get_clock()->now();
    pub_range->publish(TFmini_range); // publish data

    if(dist == -1.0) {
      RCLCPP_ERROR(node->get_logger(), "Failed to read data. TFmini node stopped!");
      break;
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  tfmini_obj->closePort();
  rclcpp::shutdown();
  return 0;
}
