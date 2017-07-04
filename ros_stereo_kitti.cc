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
#include<memory>
#include<functional>
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
#include <boost/filesystem.hpp>

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
    string packagePath = ros::package::getPath("light_mapping");

    string dictPath = packagePath + "//Vocabulary//ORBvoc.bin" ;
    string configPath = packagePath + "//config//KITTI05.yaml";

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(dictPath,configPath,ORB_SLAM2::System::STEREO, true);
    SLAM.setPublisher(nh);
    igb.mpSLAM = &SLAM ;

    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(configPath, cv::FileStorage::READ);
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

    //open the bag
    string bagPath = fsSettings["bagPath"] ;
    int imageTotalNum = fsSettings["imageTotalNum"] ;
    fsSettings.release();
    char filePath[1024] ;
    double t0 = 0 ;
    double pre_t, spend_t ;
    cv::Mat curImg(370, 1226, CV_8U ) ;
    curImg.setTo(0) ;
    cv::imshow("Current Keyframe", curImg) ;
    cv::moveWindow("Current Keyframe", 0, 700) ;
    //cv::waitKey(0) ;
    //usleep(6000000) ;
    for( int i=0 ; i <= imageTotalNum && ros::ok(); i++ )
    {
        sprintf(filePath, "%s//image_0//%06d.png",  bagPath.c_str(), i ) ;
        cv::Mat imLeft = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE );
        sprintf(filePath, "%s//image_1//%06d.png",  bagPath.c_str(), i ) ;
        cv::Mat imRight = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE );

        t0 = i ;
        pre_t = (double)cvGetTickCount() ;
        igb.mpSLAM->TrackStereo(imLeft, imRight, t0 );
        spend_t = ( (double)cvGetTickCount()- pre_t) / (cvGetTickFrequency() * 1000) ;

        while ( igb.mpSLAM->mpLocalMesher->onView && ros::ok() ){
            usleep(1000);
        }
        while ( igb.mpSLAM->mpLocalMesher->mlpKeyFrameQueue.size() > 1 && ros::ok() ){
            usleep(1000);
        }
        while ( igb.mpSLAM->mpLoopCloser->mbRunningGBA && ros::ok() ){
            usleep(1000);
        }
        //usleep(100000) ;

        double t_interval = 100 ;
        if ( spend_t < t_interval ){
            usleep((t_interval-spend_t)*1000);
        }
    }

    //std::getchar();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    //ros::shutdown();

    return 0;
}
