#ifndef __NED_WORLD_BROADCASTER_H_
#define __NED_WORLD_BROADCASTER_H_
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include "boundary_detect/Boundary.h"
#include "iarc_tf/NedWorldTransform.h"
#include "dji_sdk/LocalPosition.h"
#include "dji_sdk/AttitudeQuaternion.h"
#include "iarc_tf/Boundary.h"
#include "iarc_tf/Velocity.h"
#include <tf/tf.h>
using namespace std;
class NedWorldDynamic
{
public:
    ros::NodeHandle nh;
    ros::Subscriber dyn_local_position_sub;
    ros::Subscriber dyn_boundary_output_sub;
	ros::Subscriber dyn_local_quaternion_sub;
    string frame_name;
    geometry_msgs::Point dyn_boundary_output;
    dji_sdk::LocalPosition dyn_local_position;
	dji_sdk::AttitudeQuaternion dyn_local_quaternion;
	//tf::Quaternion dyn_local_quaternion;
    double sta_yaw;
    double sta_x, sta_y;
    double dyn_yaw;
    double dyn_x, dyn_y;
    
    NedWorldDynamic();
    ~NedWorldDynamic();
    void boundarydetectCallback(const geometry_msgs::PointConstPtr &msg);
    void localpositionCallback(const dji_sdk::LocalPositionConstPtr &msg);
	void localquaternionCallback(const dji_sdk::AttitudeQuaternionConstPtr &msg);
};



#endif