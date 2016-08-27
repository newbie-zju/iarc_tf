#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include "boundary_detect/Boundary.h"
#include "iarc_tf/NedWorldTransform.h"
#include "dji_sdk/LocalPosition.h"

#define origin_yaw 0.2
#define origin_x 1
#define origin_y 1
enum state{NOEDGE,XEDGE,YEDGE,XYEDGE};
enum xSideType{noxSide,leftSide,rightSide};
enum ySideType{noySide,topSide,bottomSide};
boundary_detect::Boundary boundary;
geometry_msgs::Point boundary_output;
dji_sdk::LocalPosition localposition_rectified;
ros::Publisher localposition_rectified_pub;
/*
void boundaryCallback(const boundary_detect::BoundaryConstPtr& msg)
{
    boundary.xSide = msg->xSide;
    boundary.ySide = msg->ySide;
    boundary.xDis = msg->xDis;
    boundary.yDis = msg->yDis;
}
*/
void boundaryoutputCallback(const geometry_msgs::PointConstPtr& msg)
{
    boundary_output.x = msg->x;
    boundary_output.y = msg->y;
    boundary_output.z = msg->z;
}

int main(int argc, char**argv){
    ros::init(argc, argv, "ned_world_listener_node");
    
    ros::NodeHandle nh;
    ros::Rate rate(1);
    //ros::Subscriber sub1 = nh.subscribe("Boundary",10,boundaryCallback);
    ros::Subscriber sub = nh.subscribe("boundary_output",10,boundaryoutputCallback);
    localposition_rectified_pub = nh.advertise<dji_sdk::LocalPosition>("/dji_sdk/local_position_rectified",10);
    /*
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion quaternion;
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"ned","world"));
    */
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
	
	if (boundary_output.z == 0.0)
	{
	    srv.request.transformState = NOEDGE;
	    srv.request.transformXDis = 0;
	    srv.request.transformYDis = 0;
	
	    if (!client.call(srv))
		ROS_INFO("transform call failed");
	    /*
	    else
	    {
		transform.setOrigin(tf::Vector3(origin_x,origin_y,0.0));
		transform.setRotation(tf::createQuaternionFromYaw(origin_yaw));
	    }
	    */
	}
    
	else if (boundary_output.z == 1.0)
	{
	    srv.request.transformState = YEDGE;
	    srv.request.transformXDis = 0.0;
	    srv.request.transformYDis = boundary_output.y;
	
	    if (!client.call(srv))
		ROS_INFO("transform call failed");
	    /*
	    else
	    {
		transform.setOrigin(tf::Vector3(srv.response.transformX,srv.response.transformY,srv.response.transformZ));
		transform.setRotation(tf::createQuaternionFromRPY(srv.response.transformRoll,srv.response.transformPitch,srv.response.transformYaw));
	    }
	    */
	}
	else if (boundary_output.z == 2.0)
	{
	    srv.request.transformState = XEDGE;
	    srv.request.transformXDis = boundary_output.x;
	    srv.request.transformYDis = 0.0;
	
	    if (!client.call(srv))
		ROS_INFO("transform call failed");
	    /*
	    else
	    {
		transform.setOrigin(tf::Vector3(srv.response.transformX,srv.response.transformY,srv.response.transformZ));
		transform.setRotation(tf::createQuaternionFromRPY(srv.response.transformRoll,srv.response.transformPitch,srv.response.transformYaw));
	    }*/
	}
	else 
	{
	    srv.request.transformState = XYEDGE;
	    srv.request.transformXDis = boundary_output.x;
	    srv.request.transformYDis = boundary_output.y;
	
	    if (!client.call(srv))
		ROS_INFO("transform call failed");
	    /*
	    else
	    {
		transform.setOrigin(tf::Vector3(srv.response.transformX,srv.response.transformY,srv.response.transformZ));
		transform.setRotation(tf::createQuaternionFromRPY(srv.response.transformRoll,srv.response.transformPitch,srv.response.transformYaw));
	    }*/
	}
	localposition_rectified.header.frame_id = "ned";
	localposition_rectified.header.stamp = ros::Time::now();
	localposition_rectified.ts = srv.response.transformts;
	localposition_rectified.x = srv.response.transformX;
	localposition_rectified.y = srv.response.transformY;
	localposition_rectified.z = srv.response.transformZ;
	localposition_rectified_pub.publish(localposition_rectified);
	
	rate.sleep();
    }
   
    
    return 0;
}