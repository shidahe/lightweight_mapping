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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

bool has_suffix(const std::string &str, const std::string &suffix) {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               bool bUseViewer):mSensor(sensor),mbReset(false),mbActivateLocalizationMode(false),
    mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
            "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
            "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
            "This is free software, and you are welcome to redistribute it" << endl <<
            "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }

    int bDisplay = fsSettings["displayViewer"] ;
    bUseViewer = bDisplay > 0 ;

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
    clock_t tStart = clock();
    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    printf("Vocabulary loaded in %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Local Meshing thread and launch
    mpLocalMesher = new LocalMeshing(strSettingsFile) ;
    mptLocalMeshing = thread(&ORB_SLAM2::LocalMeshing::Run, mpLocalMesher) ;

    //Initialize the Viewer thread and launch
    mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
    if(bUseViewer){
        mptViewer = thread(&Viewer::Run_Multi, mpViewer);
    }

    mpTracker->SetViewer(mpViewer);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);
    mpTracker->mpLocalMesher = mpLocalMesher ;

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);
    mpLocalMapper->SetLocalMesher(mpLocalMesher);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
    mpLoopCloser->SetLocalMesher(mpLocalMesher);

    mpMapDrawer->mpLocalMesher = mpLocalMesher ;
}

void System::setPublisher(ros::NodeHandle& nh)
{
    //ros publisher
    nh_ = nh ;
    mpLocalMesher->pubMesh_ = nh_.advertise<visualization_msgs::Marker>( "mesh", 1000 );
    mpLocalMesher->pubFreeSpace_ = nh_.advertise<visualization_msgs::Marker>( "freespace", 1000 );
    mpLocalMesher->pubPose_ = nh_.advertise<geometry_msgs::PoseStamped>( "pose", 100 );
    mpLocalMesher->pubObometry_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1);
    //mpLocalMesher->pubTf_ = nh_.advertise<geometry_msgs::TransformStamped>("transform", 1) ;
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    return mpTracker->GrabImageStereo(imLeft,imRight,timestamp);
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    return mpTracker->GrabImageRGBD(im,depthmap,timestamp);
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    return mpTracker->GrabImageMonocular(im,timestamp);
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMesher->mbFinished = true;
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    mpViewer->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()  ||
          !mpViewer->isFinished()      || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }
    //pangolin::BindToContext("ORB-SLAM2: Map Viewer");

    mptLocalMapping.join();
    mptLoopClosing.join();
    mptViewer.join();
    mptLocalMeshing.join();

    SaveKeyFrameAndPoints();

    if ( mpVocabulary ){
        delete mpVocabulary ;
    }
    if ( mpKeyFrameDatabase ){
        delete mpKeyFrameDatabase ;
    }
    if ( mpMap ){
        delete mpMap ;
    }
    if ( mpFrameDrawer ){
        delete mpFrameDrawer ;
    }
    if ( mpMapDrawer ){
        delete mpMapDrawer ;
    }
    if ( mpTracker ){
        delete mpTracker ;
    }
    if ( mpLocalMapper ){
        delete mpLocalMapper ;
    }
    if ( mpLoopCloser ){
        delete mpLoopCloser ;
    }
    if ( mpViewer ){
        delete mpViewer ;
    }
    if ( mpLocalMesher ){
        delete mpLocalMesher ;
    }
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameAndPoints()
{
    string filename = "KeyFrame.txt" ;
    cout << "Saving keyframe to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    cv::Mat Two = vpKFs[0]->GetPose();

    set<MapPoint*> mapPointSet ;

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;

        const vector<MapPoint*>& vpMapPoints = pKF->GetMapPointMatches();
        for( int i=0, sz = vpMapPoints.size() ; i < sz ; i++ )
        {
            MapPoint* pMP = vpMapPoints[i] ;
            if ( pMP == NULL || pMP->isBad() ){
                continue ;
            }
            mapPointSet.insert(pMP) ;
        }

        cv::Mat currentPose = Two*pKF->GetPoseInverse();
        Eigen::Matrix3d currentR ;
        Eigen::Vector3d currentT(currentPose.at<float>(0,3), currentPose.at<float>(1,3), currentPose.at<float>(2,3)) ;
        currentR << currentPose.at<float>(0,0), currentPose.at<float>(0,1), currentPose.at<float>(0,2),
                currentPose.at<float>(1,0), currentPose.at<float>(1,1), currentPose.at<float>(1,2),
                currentPose.at<float>(2,0), currentPose.at<float>(2,1), currentPose.at<float>(2,2);

        f << setprecision(6) << pKF->mnId << " " << pKF->mTimeStamp << setprecision(10)
          << " " << currentR(0, 0) << " " << currentR(0, 1) << " " << currentR(0, 2)
          << " " << currentR(1, 0) << " " << currentR(1, 1) << " " << currentR(1, 2)
          << " " << currentR(2, 0) << " " << currentR(2, 1) << " " << currentR(2, 2)
          << " " << currentT(0) << " " << currentT(1) << " " << currentT(2) << endl;
    }
    f.close();
    cout << "keyframe saved!" << endl;

    filename = "Mappoint.txt" ;
    cout << "Saving Mappoint to " << filename << " ..." << endl;
    f.open(filename.c_str());
    f << fixed;
    for( auto it: mapPointSet )
    {
        cv::Mat Xw = it->GetWorldPos();
        Xw = Two.rowRange(0,3).colRange(0,3)*Xw + Two.rowRange(0,3).col(3) ;
        f << it->mnId << setprecision(10)
          << " " << Xw.at<float>(0) << " " << Xw.at<float>(1) << " " << Xw.at<float>(2) << "\n" ;
    }
    f.close();
    cout << "Mappoint saved!" << endl;

    filename = "Constraints.txt" ;
    cout << "Saving constraints to " << filename << " ..." << endl;
    f.open(filename.c_str());
    f << fixed;
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        //KF constraints
        cv::Mat delta_T = cv::Mat::eye(4,4,CV_32F);
        f << pKF->mnId << setprecision(10)
          << " " << delta_T.at<float>(0, 0) << " " << delta_T.at<float>(0, 1) << " " << delta_T.at<float>(0, 2)
          << " " << delta_T.at<float>(1, 0) << " " << delta_T.at<float>(1, 1) << " " << delta_T.at<float>(1, 2)
          << " " << delta_T.at<float>(2, 0) << " " << delta_T.at<float>(2, 1) << " " << delta_T.at<float>(2, 2)
          << " " << delta_T.at<float>(0, 3) << " " << delta_T.at<float>(1, 3) << " " << delta_T.at<float>(2, 3) ;
        const vector<MapPoint*>& vpMapPoints = pKF->GetMapPointMatches();
        for( int i=0, sz = vpMapPoints.size() ; i < sz ; i++ )
        {
            MapPoint* pMP = vpMapPoints[i] ;
            if ( pMP == NULL || pMP->isBad() ){
                continue ;
            }
            f << " " << pMP->mnId ;
        }
        f << " " << -1 << "\n" ;

        //NON-KF constraints
        for( auto id: pKF->localFrameInfoList )
        {
            localFrameInfo tmp = mpLocalMesher->mLocalFrameInfoUnion[id] ;
            f << tmp.referenceKF->mnId << setprecision(10)
              << " " << tmp.delta_T[0][0] << " " << tmp.delta_T[0][1] << " " << tmp.delta_T[0][2]
              << " " << tmp.delta_T[1][0] << " " << tmp.delta_T[1][1] << " " << tmp.delta_T[1][2]
              << " " << tmp.delta_T[2][0] << " " << tmp.delta_T[2][1] << " " << tmp.delta_T[2][2]
              << " " << tmp.delta_T[0][3] << " " << tmp.delta_T[1][3] << " " << tmp.delta_T[2][3] ;
            for( auto vID: tmp.vertexID ){
                f << " " << vID ;
            }
            f << " " << -1 << "\n" ;
        }
    }
    f.close();
    cout << "Constraints saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

} //namespace ORB_SLAM
