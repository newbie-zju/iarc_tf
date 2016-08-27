#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include "iarc_tf/boundary_transform.h"
#include "iarc_tf/NedWorldTransform.h"

#define static_yaw 0.2
enum state{NOEDGE, XEDGE,YEDGE,XYEDGE};
float x_world,y_world;
BoundaryTransform boundarytransform;
bool poseCallback(iarc_tf::NedWorldTransform::Request &req, iarc_tf::NedWorldTransform::Response &res)
{

    switch(req.transformState)
    {
	case NOEDGE:
	    ROS_INFO("No boundary detected, use the original transform");
	    x_world = 0.0;
	    y_world = 0.0;
	    break;
	case XEDGE:
	    ROS_INFO("horizontal boundary detected, need to figure out the transform");
	    x_world = req.transformXDis;
	    y_world = 0.0;
	    break;
	case YEDGE:
	    ROS_INFO("vertical boundary detected, need to figure out the transform");
	    x_world = 0.0;
	    y_world = req.transformYDis;
	    break;
	case XYEDGE:
	    ROS_INFO("horizontal and vertical boundary detected, need to figure out the transform");
	    x_world = req.transformXDis;
	    y_world = req.transformYDis;
	    break;      
    }
    boundarytransform.getBoundaryXY(x_world,y_world);
    boundarytransform.getTransform();
    
    res.transformX = boundarytransform.p_ned.at<float>(0,0);    
    res.transformY = boundarytransform.p_ned.at<float>(1,0);
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ned_world_broadcaster_node");
    ros::NodeHandle nh;
    
    string service_name = "ned_world_transform";    
    ros::ServiceServer service = nh.advertiseService(service_name,poseCallback);
    ROS_INFO("Ready to transform new frame with world frame");
    
   
    //ros::Subscriber sub = nh.subscirbe("transform_pose",10,&poseCallback);
    
    ros::spin();
    return 0;
  
}
