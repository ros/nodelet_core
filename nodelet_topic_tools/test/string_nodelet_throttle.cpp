#include <nodelet_topic_tools/nodelet_throttle.h>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>

namespace nodelet_topic_tools {

typedef nodelet_topic_tools::NodeletThrottle<std_msgs::String> NodeletThrottleString;

}
PLUGINLIB_DECLARE_CLASS (nodelet_topic_tools, NodeletThrottleString, nodelet_topic_tools::NodeletThrottleString, nodelet::Nodelet);
