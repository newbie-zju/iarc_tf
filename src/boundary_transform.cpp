#include <math.h>
#include "iarc_tf/boundary_transform.h"

BoundaryTransform::BoundaryTransform()
{
    cyaw = 0.2;
    syaw = 0.3;
    deltax = 1.0;
    deltay = 1.0;
    
    
}

BoundaryTransform::~BoundaryTransform()
{
    ROS_INFO("boundary transform destroyed...");
}
/*
void BoundaryTransform::getBoundaryX(float x)
{
    Mat p_world = Mat::zeros(3, 1, CV_32F);
    p_world.at<float>(0,0) = x;
    //p_world.at<float>(1,0) = 
    p_world.at<float>(2,0) = 1.0;
    
}

void BoundaryTransform::getBoundaryY(float y)
{
    Mat p_world = Mat::zeros(3, 1, CV_32F);
    p_world.at<float>(1,0) = y;
    //p_world.at<float>(1,0) =
    p_world.at<float>(2,0) = 1.0;
}
*/
void BoundaryTransform::getBoundaryXY(float x, float y)
{
    p_world = Mat::zeros(3, 1, CV_32F);
    p_world.at<float>(0,0) = x;
    p_world.at<float>(1,0) = y;
    p_world.at<float>(2,0) = 1.0;
}

void BoundaryTransform::getTransform(void)
{
    Mat transform_matrix = Mat::zeros(3,3,CV_32F);
    transform_matrix.at<float>(0,0) = cyaw;
    transform_matrix.at<float>(1,0) = syaw;
    transform_matrix.at<float>(0,1) = -1.0*syaw;
    transform_matrix.at<float>(1,1) = cyaw;
    transform_matrix.at<float>(0,2) = syaw*deltay-cyaw*deltax;
    transform_matrix.at<float>(1,2) = -1.0*cyaw*deltay-syaw*deltax;
    transform_matrix.at<float>(2,2) = 1.0;
    p_ned = Mat::zeros(3, 1, CV_32F);
    
    p_ned = transform_matrix*p_world;
    
}
