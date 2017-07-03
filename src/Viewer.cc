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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];

    mViewpointCx = fSettings["Camera.cx"];
    mViewpointCy = fSettings["Camera.cy"];
}

inline float convert2realDepth( float z_far, float z_near, float z_buffer )
{
    float up = 2*z_far*z_near ;
    float down = (z_far+z_near-(z_far-z_near)*(2*z_buffer-1) ) ;
    return up/down ;
}

pangolin::OpenGlMatrix getModelViewMatrix( Eigen::Matrix4f currPose )
{
    Eigen::Matrix3f currRot = currPose.topLeftCorner(3, 3);

    Eigen::Quaternionf currQuat(currRot);
    Eigen::Vector3f forwardVector(0, 0, 1);
    Eigen::Vector3f upVector(0, -1, 0);

    Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
    Eigen::Vector3f up = (currQuat * upVector).normalized();

    Eigen::Vector3f eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));
    Eigen::Vector3f at = eye + forward;

    Eigen::Vector3f z = (eye - at).normalized();  // Forward
    Eigen::Vector3f x = up.cross(z).normalized(); // Right
    Eigen::Vector3f y = z.cross(x);

    Eigen::Matrix4d m;
    m << x(0),  x(1),  x(2),  -(x.dot(eye)),
            y(0),  y(1),  y(2),  -(y.dot(eye)),
            z(0),  z(1),  z(2),  -(z.dot(eye)),
            0,     0,     0,              1;

    pangolin::OpenGlMatrix mv;
    memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

    return mv;
}

void Viewer::Run_Multi()
{
    mbFinished = false;
    //int sceen_w = 1226 ;
    //int sceen_h = 370*2 ;
    int sceen_w = 800 ;
    int sceen_h = 650 ;
    float z_far = 2000.0 ;
    float z_near = 1 ;

    pangolin::CreateWindowAndBind("Map Viewer", sceen_w*2+160, sceen_h);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0, pangolin::Attach::Pix(sceen_w*2),pangolin::Attach::Pix(sceen_w*2+160));
    pangolin::Var<bool> switchView("menu.switch",true,true);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(sceen_w, sceen_h,mViewpointF,mViewpointF,
                                           sceen_w/2, sceen_h/2, z_near, z_far),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    pangolin::OpenGlRenderState s_cam2(
                pangolin::ProjectionMatrix(sceen_w, sceen_h,2000,2000,
                                           sceen_w/2, sceen_h/2, z_near, z_far),
                pangolin::ModelViewLookAt(mViewpointX,-1000,-0.1, 0,0,0,0.0,-1.0, 0.0)
                );


    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::Display("First Person View")
            .SetAspect((float)sceen_w/sceen_h)
            .SetBounds(0.0, 1.0, 0, 1.0)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::View& d_cam2 = pangolin::Display("Third Person View")
            .SetAspect((float)sceen_w/sceen_h)
            .SetBounds(0.0, 1.0, 0, 1.0)
            .SetHandler(new pangolin::Handler3D(s_cam2));
    pangolin::View& d_img1 = pangolin::Display("Current Image")
      .SetAspect((float)sceen_w/sceen_h);

    pangolin::Display("multi")
        .SetBounds(0.0, 1.0, 0, pangolin::Attach::Pix(sceen_w*2))
        .SetLayout(pangolin::LayoutEqualHorizontal)
        .AddDisplay(d_cam)
//        .AddDisplay(d_img1)
        .AddDisplay(d_cam2);



    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    //cv::namedWindow("ORB-SLAM2: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;

    while( !pangolin::ShouldQuit() && ros::ok() )
    {
        double t = cvGetTickCount() ;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

//        if(menuFollowCamera && bFollow)
//        {
//            s_cam.Follow(Twc);
//        }
//        else if(menuFollowCamera && !bFollow)
//        {
//            s_cam.SetModelViewMatrix(
//                        pangolin::ModelViewLookAt(0,0,0,
//                                                  mViewpointX,mViewpointY,mViewpointZ,
//                                                  0.0,-1.0, 0.0)
//                        );
//            s_cam.Follow(Twc);
//            bFollow = true;
//        }
//        else if(!menuFollowCamera && bFollow)
//        {
//            bFollow = false;
//        }

        s_cam.Follow(Twc);
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        mpMapDrawer->DrawTetrahedra(true, Twc.m[13] );
        mpMapDrawer->DrawMesh(true);
        mpMapDrawer->DrawVertexes(true);


        d_cam2.Activate(s_cam2);
        mpMapDrawer->DrawCurrentCamera(Twc);
        mpMapDrawer->DrawKeyFrames(true,true);
        if ( switchView ){
            mpMapDrawer->DrawMapPoints();
        }
        else{
            mpMapDrawer->DrawSpace(true) ;
            mpMapDrawer->DrawVertexes(true);
        }

        pangolin::FinishFrame();


//        cv::Mat im = mpFrameDrawer->DrawFrame();
//        cv::imshow("ORB-SLAM2: Current Frame",im);
//        cv::waitKey(mT);


        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;

        double renderTime = ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000) ;
        if ( renderTime < 200 ){
            usleep((200-renderTime)*1000) ;
        }
    }

    SetFinish();
}

void Viewer::Run()
{
    mbFinished = false;
    //int sceen_w = 1226 ;
    //int sceen_h = 370 ;
    int sceen_w = 1226 ;
    int sceen_h = 670 ;
    float z_far = 500.0 ;
    float z_near = 0.1 ;

    pangolin::CreateWindowAndBind("Map Viewer", sceen_w, sceen_h);


    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(160));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",false,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",false,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
    pangolin::Var<bool> menuShowVertex("menu.Show Vertex",true,true);
    pangolin::Var<bool> menuShowFace("menu.Show Face",true,true);
    pangolin::Var<bool> menuShowMesh("menu.Show Mesh",true,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    //pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);


    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(sceen_w, sceen_h,mViewpointF,mViewpointF,
                                           mViewpointCx, mViewpointCy, z_near, z_far),
//                pangolin::ProjectionMatrix(sceen_w, sceen_h,mViewpointF,mViewpointF,
//                                           sceen_w/2, sceen_h/2, z_near, z_far),
                pangolin::ModelViewLookAt(0,0,0,
                                          mViewpointX, mViewpointY, mViewpointZ,
                                          0.0, -1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -(float)sceen_w/sceen_h)
            //.SetBounds(0.0, 1.0, 0, 1.0 )
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    //cv::namedWindow("ORB-SLAM2: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;

    while(1)
    {
        double t = cvGetTickCount() ;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(
                        pangolin::ModelViewLookAt(0,0,0,
                                                  mViewpointX,mViewpointY,mViewpointZ,
                                                  0.0,-1.0, 0.0)
                        );
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

//        if(menuLocalizationMode && !bLocalizationMode)
//        {
//            mpSystem->ActivateLocalizationMode();
//            bLocalizationMode = true;
//        }
//        else if(!menuLocalizationMode && bLocalizationMode)
//        {
//            mpSystem->DeactivateLocalizationMode();
//            bLocalizationMode = false;
//        }

        d_cam.Activate(s_cam);
        glClearColor(0.9f,0.9f,0.9f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();
        if(menuShowFace){
            mpMapDrawer->DrawTetrahedra(menuShowFace, Twc.m[13] );
        }
        if (menuShowMesh){
            mpMapDrawer->DrawMesh(menuShowMesh);
        }
        if(menuShowVertex){
            mpMapDrawer->DrawVertexes(menuShowVertex);
        }

        pangolin::FinishFrame();

//        GLfloat* data = new GLfloat[sceen_w*sceen_h] ;
//        glReadPixels(0, 0, sceen_w, sceen_h, GL_DEPTH_COMPONENT, GL_FLOAT, data) ;

//        cv::Mat depthImg(sceen_h, sceen_w, CV_32F ) ;
//        cv::Mat depthImg8 ;
//        for( int i = sceen_h-1; i >= 0 ; i--)
//        {
//            for( int j = 0; j < sceen_w; j++ ){
//                depthImg.at<float>(sceen_h-1-i, j) = convert2realDepth(z_far, z_near, data[i*sceen_w+j]) ;
//            }
//        }
//        cv::normalize(depthImg, depthImg8, 0, 255, CV_MINMAX, CV_8U);
//        cv::Mat showDepthValue_cali ;
//        cv::cvtColor(depthImg8, showDepthValue_cali, CV_GRAY2RGB) ;
//        cv::putText(showDepthValue_cali, std::to_string(depthImg.at<float>(180, 500)),
//                    cv::Point2f(500, 180), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2 );
//        cv::putText(showDepthValue_cali, std::to_string(depthImg.at<float>(180, 100)),
//                    cv::Point2f(100, 180), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2 );
//        cv::putText(showDepthValue_cali, std::to_string(depthImg.at<float>(180, 300)),
//                    cv::Point2f(300, 180), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2 );
//        cv::imshow("depth", showDepthValue_cali ) ;
//        cv::waitKey(1) ;
//        delete[] data ;



//        cv::Mat im = mpFrameDrawer->DrawFrame();
//        cv::imshow("ORB-SLAM2: Current Frame",im);
//        cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            //menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;

        double renderTime = ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000) ;
        if ( renderTime < 100 ){
            usleep((100-renderTime)*1000) ;
        }
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
