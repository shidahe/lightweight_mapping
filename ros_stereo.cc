/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<list>
#include<vector>
#include<ros/ros.h>
#include"boost/thread.hpp"
#include<opencv2/core/core.hpp>

#include"src/System.h"

#include <X11/Xlib.h>

using namespace std;

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

class ImageGrabber
{
public:
    ImageGrabber(){;}
    ~ImageGrabber(){;}

    void GrabImage0(const sensor_msgs::ImageConstPtr& msg);
    void GrabImage1(const sensor_msgs::ImageConstPtr& msg);

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

void processImg()
{
    ros::Rate r(1000) ;
    bool flag ;
    while( ros::ok() )
    {
        {
            std::lock_guard<std::mutex> guard0(mMutexImg0);
            std::lock_guard<std::mutex> guard1(mMutexImg1);
            flag = image1Buf.empty() ;
            flag |= image0Buf.empty() ;
        }
        if ( flag ){
            r.sleep();
            continue ;
        }
        if ( image0Buf.begin()->t > image1Buf.begin()->t )
        {
            std::lock_guard<std::mutex> guard(mMutexImg1);
            image1Buf.pop_front();
        }
        else if ( image0Buf.begin()->t < image1Buf.begin()->t )
        {
            std::lock_guard<std::mutex> guard(mMutexImg0);
            image0Buf.pop_front();
        }
        else
        {
            cv::Mat left = image0Buf.begin()->image ;
            cv::Mat right = image1Buf.begin()->image ;
            ros::Time imgTime = image0Buf.begin()->t ;

            //double t = (double)cvGetTickCount();
            cv::Mat imLeft, imRight;
            cv::remap(left, imLeft, igb.M1l, igb.M2l, cv::INTER_LINEAR);
            cv::remap(right, imRight, igb.M1r, igb.M2r, cv::INTER_LINEAR);
            //printf("rectification time: %f\n", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000));

            igb.mpSLAM->TrackStereo(imLeft, imRight, imgTime.toSec() );
            image0Buf.pop_front();
            image1Buf.pop_front();
        }
    }
}

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

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub0 = nodeHandler.subscribe("/cam0/image_raw", 1, &ImageGrabber::GrabImage0,&igb);
    ros::Subscriber sub1 = nodeHandler.subscribe("/cam1/image_raw", 1, &ImageGrabber::GrabImage1,&igb);
    if(igb.do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }
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
    std::thread imgLoop = thread(&processImg);


    ros::spin();

    imgLoop.join();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage0(const sensor_msgs::ImageConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(mMutexImg0);
    ros::Time tImage = msg->header.stamp;
    cv::Mat image ;
    convertMsgToMatMono(msg, image) ;
    image0Buf.push_back(ImageMeasurement(tImage, image));
}

void ImageGrabber::GrabImage1(const sensor_msgs::ImageConstPtr& msg)
{

    std::lock_guard<std::mutex> guard(mMutexImg1);
    ros::Time tImage = msg->header.stamp;
    cv::Mat image ;
    convertMsgToMatMono(msg, image) ;
    image1Buf.push_back(ImageMeasurement(tImage, image));
}
