/**
 * @file local1.cpp
 * Implementation of sampling, reading data and matching.
 * You need to run the roscore and rpllidar berfore this node.
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
#include "geometry_msgs/PoseWithCovarianceStamped.h"

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

void LoadInterest(string path,unsigned int nums,vector<vector<InterestPoint>>& interestPoints);
void ReadPosition(string xpath,vector<double>& setx,
             string ypath,vector<double>& sety,
             string tpath,vector<double>& sett);
vector<double> Line2Num(string line);
void ReadingData(string filepath, vector<vector<double>>&ran, vector<vector<double>>&ang,unsigned int filenum);
void PubPose(double x,double y,double theta,ros::Publisher& my_pub);

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
unsigned int corresp[] = {0, 3, 5, 7, 9, 11, 13, 15};
double m_error[8] = {0.}, m_errorC[8] = {0.}, m_errorR[8] = {0.};
unsigned int m_match[8] = {0}, m_matchC[8] = {0}, m_matchR[8] = {0};
unsigned int m_valid[8] = {0};
unsigned int m_localSkip = 1;

vector<double> angle;
vector<double> range;
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    angle.clear();
    range.clear();
    int count = scan->scan_time / scan->time_increment;
    for(int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        degree+=180;
        angle.push_back(degree);
        range.push_back(scan->ranges[i]);
    }
}
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
    string keyFlag;
    cin>>keyFlag;
    if(keyFlag=="B")
    {
        cout<<"get it"<<endl;
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
            const LaserReading* lreadReference=new LaserReading(angle,range);
            m_detector->detect(*lreadReference, pointsPtr);
            cout<<"set ok"<<endl;
            //prepare the filennames for boost senquence
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
            //to save the interst points
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
            //save the rplidar's data
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
        }//end of the while(ros::ok()) of the data sample
    }//end of the     if(keyFlag=="B")

    //if the key input is not "B" then start the global localization
    else
    {
        //here we need to give the number of files with .dat in the "data/inner/interest"
        unsigned int filenums=443;
        ofstream localization("data/inner/localization.txt");
                 localization<<"real position                 "<<"detect position"<<endl;
        ofstream matlabresult("data/inner/matlabresult.txt");
        matlabresult<<"real(x,y)    "<<"get(x,y)    "<<"inliners"<<endl;
        ros::init(argc, argv, "example");
        ros::NodeHandle n;
        ros::Rate loop_rate(1);      //control the frequency
        ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
        ros::Subscriber subodom=n.subscribe<nav_msgs::Odometry>("/odom", 1000, odomCallback);
        ros::Publisher pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1,true);
        cout<<"A"<<endl;
        std::vector< std::vector<InterestPoint *> > m_pointsReference;
        vector<vector<InterestPoint>> m_points;
        string filepath="data/inner/interest/";//the data of the interest point
        string filenameoflidar="data/inner/interest/";//the data of the rplidar
        //read the data of interest points
        LoadInterest(filepath,filenums,m_points);
        for(unsigned int i=0;i<m_points.size();i++)
        {
            vector<InterestPoint *> tempPtr;
            for(unsigned int j=0;j<m_points[i].size();j++)
            {
                tempPtr.push_back(&(m_points[i][j]));
            }
            m_pointsReference.push_back(tempPtr);
        }
        //read the data of rplidar
        vector<vector<double>>setrange;
        vector<vector<double>>setangle;
        ReadingData(filenameoflidar, setrange, setangle,filenums);
        //read the data of position
        vector<double> setx;
        vector<double> sety;
        vector<double> sett;
        ReadPosition("data/inner/px.txt",setx,
                     "data/inner/py.txt",sety,
                     "data/inner/pt.txt",sett);
        unsigned int count=0;
        cout<<"ok: get dataset"<<endl;
        //distance
        HistogramDistance<double> *dist = NULL;
        dist = new SymmetricChi2Distance<double>();
        //dist = new EuclideanDistance<double>();
        //descriptor
        m_betaGenerator = new BetaGridGenerator(0.02, 0.5, 4, 12);
        m_betaGenerator->setDistanceFunction(dist);
        m_descriptor = m_betaGenerator;
        for(unsigned int i=0;i<m_pointsReference.size();i++)
        {
            const LaserReading* lreadReference=new LaserReading(setangle[i],setrange[i]);
            for(unsigned int j=0;j<m_pointsReference[i].size();j++)
            {
                m_pointsReference[i][j]->setDescriptor(m_descriptor->describe(*m_pointsReference[i][j], *lreadReference));
            }
        }
        while(ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
            unsigned int testpos=100;//use for test the match by set with set

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
            //dist = new EuclideanDistance<double>();
            //descriptor
            m_betaGenerator = new BetaGridGenerator(0.02, 0.5, 4, 12);
            m_betaGenerator->setDistanceFunction(dist);
            m_descriptor = m_betaGenerator;
            //stratefy
            m_ransac = new RansacFeatureSetMatcher(acceptanceSigma * acceptanceSigma * 5.99, success, inlier, matchingThreshold, acceptanceSigma * acceptanceSigma * 3.84, false);
            cout<<"ok: set deccriptor"<<endl;
            cout<<"HELLO"<<endl;
            //initialization
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
            for(unsigned int i=0;i<pointsPtr.size();i++){
                pointsPtr[i]->setDescriptor(m_descriptor->describe(*pointsPtr[i], *lreadReference));
            }//set descriptor
            cout<<"laser interest pos: "<<pointsPtr[0]->getPosition().x<<endl;
            cout<<"ok: get laser interst"<<endl;
            cout<<"real position:"<<"("<<odomx<<","<<odomy<<")"<<endl;
            localization<<"count"<<count<<'\t';
            localization<<"        ("<<odomx<<","<<odomy<<")"<<'\t';
            matlabresult<<odomx<<"   "<<odomy<<"   ";
            unsigned int inliers[m_pointsReference.size()];
            double results[m_pointsReference.size()];
            for(unsigned int i=0;i<m_pointsReference.size();i++){
                results[i] = 1e17;
                inliers[i] = 0;
            }
            for(unsigned int i=0;i<m_pointsReference.size();i++)
            {
                OrientedPoint2D transform;
                std::vector< std::pair<InterestPoint*, InterestPoint* > > correspondences(m_pointsReference[i].size());
                //match to get the inliners
                results[i]= m_ransac->matchSets(pointsPtr, m_pointsReference[i], transform, correspondences);
                inliers[i] = correspondences.size();
            }
            cout<<"ok: finish match"<<endl;
            unsigned int maxnum=0;
            for(unsigned int i=0;i<m_pointsReference.size();i++)
            {
                if(inliers[i]>maxnum)
                    maxnum=i;
            }
            cout<<"maxinlier is "<<inliers[maxnum]<<endl;
            cout<<"detectposition is ("<<setx[maxnum]<<", "<<sety[maxnum]<<")"<<endl;
            localization<<"   ("<<setx[maxnum]<<", "<<sety[maxnum]<<")"<<"    "<<"num of maxliners is "<<inliers[maxnum]<<endl;
            matlabresult<<setx[maxnum]<<"   "<<sety[maxnum]<<"   "<<inliers[maxnum]<<endl;
            //if the number of the inliers larger than 12, we publish the position
            if(inliers[maxnum]>=12){
                PubPose(setx[maxnum],sety[maxnum],0,pub);
            }
            delete lreadReference;
            delete m_detectorCurvature;
            delete dist;
            delete m_betaGenerator;
            delete m_ransac;
            count++;
        }    //end of while(ros::ok())
    }//end of the else of if(keyFlag=="B")
return 0;
}//end of main()

//used to load the data from the path, to get interst points' data
void LoadInterest(string path,unsigned int nums,vector<vector<InterestPoint>>& interestPoints)
{
    vector<string> names;
    for(unsigned int i=1;i<=nums;i++)
    {
        string filename=path;
        string  temp;
        stringstream a;
        a<<i;
        a>>temp;
        temp+=".dat";
        filename+=temp;
        names.push_back(filename);
    }
    for(unsigned int i=0;i<nums;i++)
    {
        ifstream rf(names[i]);
        boost::archive::binary_iarchive boostrf(rf);
        vector<InterestPoint> pointstemp;
        InterestPoint point;
        string num;
        boostrf>>num;
        stringstream ss;
        unsigned int j;
        ss<<num;
        ss>>j;
        //cout<<"j "<<j<<endl;
        for(unsigned int i=0;i<j;i++){
            boostrf>>point;
            pointstemp.push_back(point);
        }
        interestPoints.push_back(pointstemp);
        rf.close();
    }
}
//used to read position data from xpath
void ReadPosition(string xpath,vector<double>& setx,
             string ypath,vector<double>& sety,
             string tpath,vector<double>& sett)
{
    ifstream xf(xpath);
    ifstream yf(ypath);
    ifstream tf(tpath);
    string tempstring;
    while(getline(xf,tempstring)){
        stringstream ss(tempstring);
        double temp;
        ss>>temp;
        setx.push_back(temp);
    }
    while(getline(yf,tempstring)){
        stringstream ss(tempstring);
        double temp;
        ss>>temp;
        sety.push_back(temp);
    }
    while(getline(tf,tempstring)){
        stringstream ss(tempstring);
        double temp;
        ss>>temp;
        sett.push_back(temp);
    }
    xf.close();
    yf.close();
    tf.close();
}
//used to get rplidar's data from file path
void ReadingData(string filepath, vector<vector<double>>&ran, vector<vector<double>>&ang,unsigned int filenum)
{
    vector<string> names;
    for(unsigned int i=1;i<=filenum;i++)
    {
        string filename=filepath;
        string  temp;
        stringstream a;
        a<<i;
        a>>temp;
        temp+=".txt";
        filename+=temp;
        names.push_back(filename);
    }
    for(unsigned int i=0;i<filenum;i++)
    {
        ifstream inFile(names[i]);
        string line;
        while(getline(inFile,line))//detect the first laser
        {
            string temp;
            stringstream input(line);
            input>>temp;
            if(temp=="frameid")
                break;
        }
        while(getline(inFile,line))
        {
            vector<double> rangetemp;
            vector<double> angletemp;
            string firstword;
            stringstream input(line);
            input>>firstword;
            if(firstword=="r"){
                rangetemp=Line2Num(line);
                ran.push_back(rangetemp);
            }
            if(firstword=="a"){
                angletemp=Line2Num(line);
                ang.push_back(angletemp);
            }
        }
        inFile.close();
    }
}
//used to transform a line of string to number
vector<double> Line2Num(string line)
{
    vector<double> range;
    string word;
    stringstream input(line);
    while(input>>word)
    {
        if(word=="r"||word=="a")//to jump the r and a
            continue;
        if(word=="inf")//to deal the inf
            range.push_back(1e17);
        else          //to deal the number
        {
            double num;
            istringstream ss(word);
            ss>>num;
            range.push_back(num);
        }
    }
    return range;
}
//used to publish the position which we geted
void PubPose(double x,double y,double theta,ros::Publisher& my_pub)
{
    geometry_msgs::PoseWithCovarianceStamped from_abs_pt;
    from_abs_pt.header.frame_id="map";
    from_abs_pt.pose.pose.position.x=x;
    from_abs_pt.pose.pose.position.y=y;
    from_abs_pt.pose.pose.position.z=theta;
    from_abs_pt.pose.covariance[0]=0.05;
    from_abs_pt.pose.covariance[7]=0.05;
    from_abs_pt.pose.covariance[35]=0.05;
    my_pub.publish(from_abs_pt);
}
