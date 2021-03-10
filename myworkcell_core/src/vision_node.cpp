/**
** Simple ROS Node
**/
#include <ros/ros.h>
#include <fake_ar_publisher/ARMarker.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>

class Localizer
{
public:

  ros::Subscriber ar_sub_;
  ros::ServiceServer server_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  fake_ar_publisher::ARMarkerConstPtr last_msg_;
  geometry_msgs::PoseStamped target_pose_from_cam;
  geometry_msgs::PoseStamped target_pose_from_req;


  Localizer(ros::NodeHandle& nh) : listener_(buffer_)
  {  
     server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
     ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1, &Localizer::visionCallback, this);
  }
  
  bool localizePart(myworkcell_core::LocalizePart::Request& req,
                    myworkcell_core::LocalizePart::Response& res)
  {
     fake_ar_publisher::ARMarkerConstPtr p = last_msg_;

     if (!p) return false;
     
     target_pose_from_cam.header = p->header;
     target_pose_from_cam.pose = p->pose.pose;
     target_pose_from_req = buffer_.transform(target_pose_from_cam, req.base_frame);
     
     //res.pose = p ->pose.pose;
     res.pose = target_pose_from_req.pose;
     return true;

  }

  void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)
  {
     last_msg_ = msg;
//     ROS_INFO_STREAM(last_msg_ ->pose.pose);
  }


};

int main(int argc, char* argv[])
{
	
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;
     
        Localizer localizer(nh);

	ROS_INFO("Vision node starting");

	ros::spin();
}
