/**
 * @file datainnersample.cpp
 * Implementation of sampling and savin data.
 * You need to run the roscore and rpllidar berfore this node
 * @author dijian
 * @time 2017.9.4 Guangzhou
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

#include <feature/Detector.h>
#include <feature/ShapeContext.h>
#include <feature/BetaGrid.h>
#include <feature/RangeDetector.h>
#include <feature/CurvatureDetector.h>
#include <feature/NormalBlobDetector.h>
#include <feature/NormalEdgeDetector.h>
#include <feature/RansacFeatureSetMatcher.h>
#include <feature/RansacMultiFeatureSetMatcher.h>
#include <sensorstream/CarmenLog.h>
#include <sensorstream/LogSensorStream.h>
#include <sensorstream/SensorStream.h>
#include <utils/SimpleMinMaxPeakFinder.h>
#include <utils/HistogramDistances.h>

#include <stdlib.h>
#include <iostream>
#include <string>
#include <string.h>
#include <sstream>
#include <utility>
#include <sys/time.h>
#include <vector>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
using namespace std;
#define RAD2DEG(x) ((x)*180./M_PI)
#define CV_PI   3.1415926535897932384626433832795

CurvatureDetector *m_detectorCurvature = NULL;    //four kinds of detect
NormalBlobDetector *m_detectorNormalBlob = NULL;
NormalEdgeDetector *m_detectorNormalEdge = NULL;
RangeDetector *m_detectorRange = NULL;
Detector* m_detector = NULL;
BetaGridGenerator *m_betaGenerator = NULL;         //description
ShapeContextGenerator *m_shapeGenerator = NULL;
DescriptorGenerator *m_descriptor = NULL;
RansacFeatureSetMatcher *m_ransac = NULL;

double angErrorTh = 0.2;
double linErrorTh = 0.5;
vector<vector<InterestPoint>> m_Point;
std::vector< std::vector<InterestPoint *> > m_pointsReference;
std::vector< OrientedPoint2D > m_posesReference;
unsigned int corresp[] = {0, 3, 5, 7, 9, 11, 13, 15};
double m_error[8] = {0.}, m_errorC[8] = {0.}, m_errorR[8] = {0.};
unsigned int m_match[8] = {0}, m_matchC[8] = {0}, m_matchR[8] = {0};
unsigned int m_valid[8] = {0};
struct timeval detectTime, describeTime, ransacTime;
unsigned int m_localSkip = 1;
//subscribe the data of scan
vector<double> angle;
vector<double> range;
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    angle.clear();
    range.clear();
    int count = scan->scan_time / scan->time_increment;
    for(int i = 0; i < count; i++){
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        degree+=180;
        angle.push_back(degree);
        range.push_back(scan->ranges[i]);
    }
}
//subscribe the data of odom
double odomx;
double odomy;
double odomt;
void odomCallback(const nav_msgs::Odometry::ConstPtr&msg)
{
    geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);
    odomx = msg->pose.pose.position.x;
    odomy = msg->pose.pose.position.y;
    odomt = (yaw/CV_PI)*180.0;
}

int main(int argc, char **argv)
{

    ofstream interest("data/inner/interest.txt");
    ofstream px("data/inner/px.txt");
    ofstream py("data/inner/py.txt");
    ofstream pt("data/inner/pt.txt");

    ros::init(argc, argv, "example");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);      //control the frequency
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    ros::Subscriber subodom=n.subscribe<nav_msgs::Odometry>("/odom", 1000, odomCallback);
    cout<<"A"<<endl;
    unsigned int count=0;
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        if(angle.size()==0)
            continue;
        unsigned int scale = 5, dmst = 2, window = 3;
        double baseSigma = 0.2, sigmaStep = 1.4, minPeak = 0.34, minPeakDistance = 0.001, acceptanceSigma = 0.1, success = 0.95, inlier = 0.4, matchingThreshold = 0.4;
        bool useMaxRange = false;
        SimpleMinMaxPeakFinder *m_peakMinMax = new SimpleMinMaxPeakFinder(minPeak, minPeakDistance);
        ROS_INFO_STREAM("B"<<endl);
        //detector
        m_detectorCurvature = new CurvatureDetector(m_peakMinMax, scale, baseSigma, sigmaStep, dmst);
        m_detectorCurvature->setUseMaxRange(useMaxRange);
        m_detector = m_detectorCurvature;
        //distance
        HistogramDistance<double> *dist = NULL;
        dist = new SymmetricChi2Distance<double>();
        //descriptor
        m_betaGenerator = new BetaGridGenerator(0.02, 0.5, 4, 12);
        m_betaGenerator->setDistanceFunction(dist);
        m_descriptor = m_betaGenerator;
        //stratefy
        m_ransac = new RansacFeatureSetMatcher(acceptanceSigma * acceptanceSigma * 5.99, success, inlier, matchingThreshold, acceptanceSigma * acceptanceSigma * 3.84, false);
        cout<<"HELLO"<<endl;
        vector<InterestPoint>   points;
        vector<InterestPoint *> pointsPtr;
        for(unsigned int i=0;i<range.size();i++){
            InterestPoint temp;
            points.push_back(temp);
        }
        for(unsigned int i=0;i<range.size();i++){
            InterestPoint * temp;
            pointsPtr.push_back(temp);
        }
        for(unsigned int i=0;i<range.size();i++){
            pointsPtr[i]=&points[i];
        }
        //get interest points
        const LaserReading* lreadReference=new LaserReading(angle,range);
        m_detector->detect(*lreadReference, pointsPtr);
        cout<<"set ok"<<endl;
        //prepare the file names
        count++;
        string  temp;
        string filename="data/inner/interest/";
        string rp;
        string rpfilename="data/inner/interest/";
        stringstream a;
        a<<count;
        a>>temp;
        rp=temp;
        temp+=".dat";
        rp+=".txt";
        filename+=temp;
        rpfilename+=rp;
        //save the data
        ofstream of(filename);
        boost::archive::binary_oarchive boostof(of);
        string num;
        stringstream ss;
        ss<<pointsPtr.size();
        ss>>num;
        boostof<<num;
        interest<<"size: "<<pointsPtr.size();
        for(unsigned int i=0;i<pointsPtr.size();i++)
        {
            boostof<<*(pointsPtr[i]);

            interest<<"("<<pointsPtr[i]->getPosition().x<<", "
                    <<pointsPtr[i]->getPosition().y<<")"<<'\t';
        }
        interest<<endl;
        interest<<endl;
        of.close();
        px<<odomx<<endl;
        py<<odomy<<endl;
        pt<<odomt<<endl;
        ofstream saveFile(rpfilename);
        saveFile<<"frameid"<<" "<<count<<endl;
        saveFile<<"r ";
        for(unsigned int i=0;i<range.size();i++){
            saveFile<<range[i]<<" ";
        }
        saveFile<<endl;
        saveFile<<"a ";
        for(unsigned int i=0;i<angle.size();i++){
            saveFile<<angle[i]<<" ";
        }
        saveFile<<endl;
        saveFile<<endl;
        saveFile.close();
        cout<<"count: "<<count<<endl;
        delete lreadReference;
        delete m_detectorCurvature;
        delete dist;
        delete m_betaGenerator;
    }
    return 0;
}
