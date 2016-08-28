#include "iarc_tf/ned_world_dynamic.h"

using namespace std;
NedWorldDynamic::NedWorldDynamic():nh("~")
{
    sta_yaw = -0.136;
    sta_x = 0.0;
    sta_y = 4.0;
    dyn_local_position_sub = nh.subscribe("/dji_sdk/local_position",10,&NedWorldDynamic::localpositionCallback,this);
    dyn_boundary_output_sub = nh.subscribe("/boundary_output",10,&NedWorldDynamic::boundarydetectCallback,this);
    
}

NedWorldDynamic::~NedWorldDynamic()
{
    ROS_INFO("destroying the ned_world_dynamic node ...");
}
void NedWorldDynamic::localpositionCallback(const dji_sdk::LocalPositionConstPtr& msg)
{
    //ROS_INFO("TEST..");
    dyn_local_position.x = msg->x;
    dyn_local_position.y = msg->y;
    dyn_local_position.z = msg->z;
    
    ROS_INFO_STREAM("dyn_local_position_x: "<<dyn_local_position.x);
    
}

void NedWorldDynamic::boundarydetectCallback(const geometry_msgs::PointConstPtr& msg)
{
    dyn_boundary_output.x = msg->x;
    dyn_boundary_output.y = msg->y;
    dyn_boundary_output.z = msg->z;
    ROS_INFO_STREAM("dyn_boundary_z: "<<dyn_boundary_output.z);
    
    switch(int(dyn_boundary_output.z))
    {
	case 0:
	    dyn_yaw = sta_yaw;
	    dyn_x = sta_x;
	    dyn_y = sta_y;
	    ROS_INFO("NOEDGE");
	    break;
	case 1:
	    dyn_yaw = sta_yaw;
	    dyn_x = sta_x;
	    dyn_y = dyn_boundary_output.y+sin(dyn_yaw)*dyn_local_position.x-cos(dyn_yaw)*dyn_local_position.y;
	    ROS_INFO("XEDGE");
	    break;
	case 2:
	    dyn_yaw = sta_yaw;
	    dyn_y = sta_y;
	    dyn_x = dyn_boundary_output.x-sin(dyn_yaw)*dyn_local_position.y-cos(dyn_yaw)*dyn_local_position.x;
	    ROS_INFO("YEDGE");
	    break;
	case 3:
	    dyn_yaw = sta_yaw;
	    dyn_x = dyn_boundary_output.x-sin(dyn_yaw)*dyn_local_position.y-cos(dyn_yaw)*dyn_local_position.x;
	    dyn_y = dyn_boundary_output.y+sin(dyn_yaw)*dyn_local_position.x-cos(dyn_yaw)*dyn_local_position.y;
	    ROS_INFO("XYEDGE");
	    break;
    }
    
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(dyn_x, dyn_y,0.0));
    tf::Quaternion quaternion;
    transform.setRotation(tf::createQuaternionFromYaw(dyn_yaw));
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world","ground"));
}


