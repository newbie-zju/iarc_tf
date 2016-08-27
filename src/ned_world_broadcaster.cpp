#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include "boundary_detect/Boundary.h"
#include "iarc_tf/NedWorldTransform.h"

#define static_yaw 0.2
enum state{NOEDGE,XEDGE,YEDGE,XYEDGE};
enum xSideType{noxSide,leftSide,rightSide};
enum ySideType{noySide,topSide,bottomSide};
boundary_detect::Boundary boundary;

void boundaryCallback(const boundary_detect::BoundaryConstPtr& msg)
{
    boundary.xSide = msg->xSide;
    boundary.ySide = msg->ySide;
    boundary.xDis = msg->xDis;
    boundary.yDis = msg->yDis;
}

int main(int argc, char**argv){
    ros::init(argc, argv, "ned_world_listener_node");
    
    ros::NodeHandle nh;
    ros::Rate rate(1);
    ros::Subscriber sub = nh.subscribe("Boundary",10,boundaryCallback);
    
    
    
    while(!ros::service::waitForService("ned_world_transform",ros::Duration(3.0)))
    {
	ROS_INFO("Waiting for service ned_world_transform to become available");
    }
    ROS_INFO("Requseting the transform ...");
    ros::ServiceClient client = nh.serviceClient<iarc_tf::NedWorldTransform>("ned_world_transform");
    iarc_tf::NedWorldTransform srv;
    
    while(ros::ok())
    {
	ros::spinOnce();
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion quaternion;
	if (boundary.xSide == noxSide && boundary.ySide == noySide)
	{
	    srv.request.transformState = NOEDGE;
	    srv.request.transformXDis = 0.0;
	    srv.request.transformYDis = 0.0;
	
	    if (!client.call(srv))
		ROS_INFO("transform call failed");
	    else
	    {
		transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
		transform.setRotation(tf::createQuaternionFromYaw(0.2));
	    }
	}
    
	else if (boundary.xSide == noxSide && boundary.ySide != noySide)
	{
	    srv.request.transformState = YEDGE;
	    srv.request.transformXDis = 0.0;
	    srv.request.transformYDis = boundary.yDis;
	
	    if (!client.call(srv))
		ROS_INFO("transform call failed");
	    else
	    {
		transform.setOrigin(tf::Vector3(srv.response.transformX,srv.response.transformY,srv.response.transformZ));
		transform.setRotation(tf::createQuaternionFromRPY(srv.response.transformRoll,srv.response.transformPitch,srv.response.transformYaw));
	    }
	}
	else if (boundary.xSide != noxSide && boundary.ySide == noySide)
	{
	    srv.request.transformState = XEDGE;
	    srv.request.transformXDis = boundary.xDis;
	    srv.request.transformYDis = 0.0;
	
	    if (!client.call(srv))
		ROS_INFO("transform call failed");
	    else
	    {
		transform.setOrigin(tf::Vector3(srv.response.transformX,srv.response.transformY,srv.response.transformZ));
		transform.setRotation(tf::createQuaternionFromRPY(srv.response.transformRoll,srv.response.transformPitch,srv.response.transformYaw));
	    }
	}
	else 
	{
	    srv.request.transformState = XYEDGE;
	    srv.request.transformXDis = boundary.xDis;
	    srv.request.transformYDis = boundary.yDis;
	
	    if (!client.call(srv))
		ROS_INFO("transform call failed");
	    else
	    {
		transform.setOrigin(tf::Vector3(srv.response.transformX,srv.response.transformY,srv.response.transformZ));
		transform.setRotation(tf::createQuaternionFromRPY(srv.response.transformRoll,srv.response.transformPitch,srv.response.transformYaw));
	    }
	}
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"ned","world"));
    rate.sleep();
    }
   
    
    return 0;
}