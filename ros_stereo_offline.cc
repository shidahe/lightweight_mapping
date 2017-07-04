/**
* This file is part of the implementation of our paper: Yonggen Ling and Shaojie Shen, "Building Maps for Autonomous Navigation Using Sparse Visual SLAM Features" in IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2017. 
*
* For more information see <https://github.com/ygling2008/lightweight_mapping>
*
* This code is a free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This code is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this code. If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <memory>
#include <functional>
#include<list>
#include<vector>
#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include"boost/thread.hpp"
#include<opencv2/core/core.hpp>
#include<rosbag/bag.h>
#include<rosbag/chunked_file.h>
#include<rosbag/view.h>
#include<rosbag/query.h>
#include"src/System.h"
#include <stdio.h>
#include <X11/Xlib.h>

void convertMsgToMatMono(sensor_msgs::ImageConstPtr& msg, cv::Mat& img)
{
    int width = msg->width ;
    int height = msg->height ;
    img = cv::Mat(height, width, CV_8U);
    int k = 0 ;
    for( int i = 0 ; i < height ; i++ )
    {
        for ( int j = 0 ; j < width ; j++ )
        {
            img.at<uchar>(i, j) = msg->data[k] ;
            k++ ;
        }
    }
}


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(){;}
    ~ImageGrabber(){;}


    ORB_SLAM2::System* mpSLAM;
    bool do_rectify = true;
    cv::Mat M1l,M2l,M1r,M2r;
}igb;

class ImageMeasurement
{
public:
    ros::Time t;
    cv::Mat   image;

    ImageMeasurement(const ros::Time& _t, const cv::Mat& _image)
    {
        t     = _t;
        image = _image.clone();
    }

    ImageMeasurement(const ImageMeasurement& i)
    {
        t     = i.t;
        image = i.image.clone();
    }

    ~ImageMeasurement() { ;}
};

std::list<ImageMeasurement> image0Buf;
std::list<ImageMeasurement> image1Buf;
std::mutex mMutexImg0;
std::mutex mMutexImg1;
ORB_SLAM2::System* pSystem ;

int main(int argc, char **argv)
{
    XInitThreads();

    ros::init(argc, argv, "Stereo");
    ros::start();
    ros::NodeHandle nh("~") ;

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 stereo path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO, true);
    SLAM.setPublisher(nh);
    igb.mpSLAM = &SLAM ;

    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }
    int preRectified = fsSettings["preRectified"] ;
    if ( preRectified > 0 ){
        igb.do_rectify = false ;
    }
    else{
        igb.do_rectify = true ;
    }
    if(igb.do_rectify)
    {
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);

        igb.mpSLAM->mpLocalMesher->M1l = igb.M1l.clone();
        igb.mpSLAM->mpLocalMesher->M1r = igb.M1r.clone();
        igb.mpSLAM->mpLocalMesher->M2l = igb.M2l.clone();
        igb.mpSLAM->mpLocalMesher->M2r = igb.M2r.clone();
    }

    int flipStereo = fsSettings["flipStereo"];

    //open the bag
    string bagPath = fsSettings["bagPath"] ;
    rosbag::Bag bag(bagPath, rosbag::bagmode::Read);

    double startTime = fsSettings["startTime"] ;

    std::string imu_topic("/imu0");
    rosbag::View view_imu(
                bag,
                rosbag::TopicQuery(imu_topic));

    std::string camera_topic ;
    if ( flipStereo > 0 ){
        camera_topic = "/cam"+std::to_string(1)+"/image_raw" ;
    }
    else {
        camera_topic = "/cam"+std::to_string(0)+"/image_raw" ;
    }
    rosbag::View view_image0(
                bag,
                rosbag::TopicQuery(camera_topic));

    if ( flipStereo > 0 ){
        camera_topic = "/cam"+std::to_string(0)+"/image_raw" ;
    }
    else {
        camera_topic = "/cam"+std::to_string(1)+"/image_raw" ;
    }
    rosbag::View view_image1(
                bag,
                rosbag::TopicQuery(camera_topic));

    rosbag::View::iterator view_cam_iter0 = view_image0.begin() ;
    rosbag::View::iterator view_cam_iter1 = view_image1.begin() ;


    bool init = false ;
    while( ros::ok() )
    {
        if ( view_cam_iter0 == view_image0.end() ){
            break ;
        }
        if ( view_cam_iter1 == view_image1.end() ){
            break ;
        }
        sensor_msgs::ImageConstPtr msg0 = view_cam_iter0
                ->instantiate<sensor_msgs::Image>();

        sensor_msgs::ImageConstPtr msg1 = view_cam_iter1
                ->instantiate<sensor_msgs::Image>();

        double t0 = msg0->header.stamp.toSec() ;
        double t1 = msg1->header.stamp.toSec() ;

        if ( init == false )
        {
            if ( t0 > t1 ){
                startTime += t0 ;
            }
            else {
                startTime += t1 ;
            }
            init = true ;
        }

        if ( t0 < startTime ){
            view_cam_iter0++ ;
            continue ;
        }
        if ( t1 < startTime ){
            view_cam_iter1++ ;
            continue ;
        }
        //        std::cout << msg0->header.stamp << " " << msg1->header.stamp << "\n" ;



        // cv::Mat left = cv_bridge::toCvShare(msg0, std::string("mono8"))->image;
        // cv::Mat right = cv_bridge::toCvShare(msg1, std::string("mono8"))->image;
        cv::Mat left, right ;
        convertMsgToMatMono(msg0, left) ;
        convertMsgToMatMono(msg1, right) ;
        
        //double t = (double)cvGetTickCount();
        cv::Mat imLeft, imRight;
        if ( igb.do_rectify ){
            cv::remap(left, imLeft, igb.M1l, igb.M2l, cv::INTER_LINEAR);
            cv::remap(right, imRight, igb.M1r, igb.M2r, cv::INTER_LINEAR);
        }
        else {
            imLeft = left.clone();
            imRight = right.clone();
        }
        //printf("rectification time: %f\n", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000));

        igb.mpSLAM->TrackStereo(imLeft, imRight, t0 );

        view_cam_iter0++ ;
        view_cam_iter1++ ;

        while ( igb.mpSLAM->mpLocalMesher->onView == true ){
            usleep(1000); ;
        }
    }

    std::getchar();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}
