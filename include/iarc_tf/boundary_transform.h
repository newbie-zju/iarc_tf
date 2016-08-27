#ifndef __BOUNDARY_TRANSFORM_H_
#define __BOUNDARY_TRANSFORM_H_

#include <ros/ros.h>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

class BoundaryTransform
{
public:
    float cyaw,syaw;
    float deltax,deltay;
    Mat p_world;
    Mat p_ned;
    BoundaryTransform();
    ~BoundaryTransform();
    
    void getBoundaryX(float x);
    void getBoundaryY(float y);
    void getBoundaryXY(float x, float y);
    void getTransform(void);
    
};


#endif 