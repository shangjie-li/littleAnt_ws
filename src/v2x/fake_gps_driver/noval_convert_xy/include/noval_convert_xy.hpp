
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
 

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "fake_gps_msgs/Ephemeris.h"
#include "fake_gps_msgs/L1L2Range.h"
#include "fake_gps_msgs/Inspvax.h"   //add by wendao
#include "sensor_msgs/Imu.h"

#include <boost/tokenizer.hpp>
#include <boost/thread/thread.hpp>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPoint.h>
#include <Eigen/Dense>

using namespace std;
class noval_convert_xy
{
    public:
    void jwd_to_xy(double jd,double wd,double& x,double& y);
};
void noval_convert_xy::jwd_to_xy(double jd,double wd,double& x,double& y)
{
    geographic_msgs::GeoPoint point;

    //117.4375922, 39.1921173

    point.latitude = 117.4375922;
    point.longitude = 39.1921173;
    point.altitude = 0;

    geodesy::UTMPoint utm;
    geodesy::fromMsg(point, utm);

    //east-north-sky
    // ll2utm_msg.pose.pose.position.x = utm.easting;
    // ll2utm_msg.pose.pose.position.y = utm.northing;
    // ll2utm_msg.pose.pose.position.z = utm.altitude;

    std::cout<<"x ";
    printf("%15f",utm.easting);
    std::cout<<endl;
    std::cout<<"y ";
    printf("%15f",utm.northing);
    std::cout<<endl;
    x=utm.easting;
    y=utm.northing;
}
