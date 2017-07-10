#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Time.h>

namespace test_nodelet
{

class NodehandleTest : public nodelet::Nodelet
{
public:
  NodehandleTest(){};
  virtual void onInit()
  {
    ros::NodeHandle nh = this->getNodeHandle();
    ros::NodeHandle pnh = this->getPrivateNodeHandle();
    global_pub_ = nh.advertise<std_msgs::Time>("/global", 1000);
    namespaced_pub_ = nh.advertise<std_msgs::Byte>("namespaced", 1000);
    private_pub_ = pnh.advertise<std_msgs::Bool>("private", 1000);
  }
private:
  ros::Publisher global_pub_, namespaced_pub_, private_pub_;
};

} // namespace dude

PLUGINLIB_EXPORT_CLASS(test_nodelet::NodehandleTest, nodelet::Nodelet);
