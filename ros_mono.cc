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
#include <cv_bridge/cv_bridge.h>
#include "boost/thread.hpp"
#include<opencv2/core/core.hpp>

#include"src/System.h"

#include <X11/Xlib.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

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
            std::lock_guard<std::mutex> guard(mMutexImg0);
            flag = image0Buf.empty() ;
        }
        if ( flag ){
            r.sleep();
        }
        else {
            cv::Mat img = image0Buf.begin()->image ;
            ros::Time t = image0Buf.begin()->t ;
            pSystem->TrackMonocular(img, t.toSec());
            image0Buf.pop_front();
        }

    }
}

int main(int argc, char **argv)
{
    XInitThreads();

    ros::init(argc, argv, "Mono");
    ros::start();
    ros::NodeHandle nh("~") ;

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, true);
    SLAM.setPublisher(nh);
    pSystem = &SLAM ;
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/cam0/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    std::thread imgLoop = thread(&processImg); ;

    ros::spin();

    imgLoop.join();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::lock_guard<std::mutex> guard(mMutexImg0);
    ros::Time tImage = msg->header.stamp;
    cv::Mat image  = cv_bridge::toCvShare(msg, std::string("mono8"))->image;
    image0Buf.push_back(ImageMeasurement(tImage, image));
}
