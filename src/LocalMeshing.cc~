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


#include "LocalMeshing.h"

namespace ORB_SLAM2
{

inline void updateXYbound( double x, double y, int& lx, int& rx, int& ly, int& ry )
{
    if ( x < lx ){
        lx = x ;
    }
    if ( y < ly ){
        ly = y ;
    }
    if ( x > rx ){
        rx = x ;
    }
    if ( y > ry ){
        ry = y ;
    }
}

LocalMeshing::LocalMeshing(const string &strSettingPath)
{
    cv::FileStorage fsSettings(strSettingPath.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "Failed to open settings file at: " << strSettingPath << std::endl;
        exit(-1);
    }

    img_height = fsSettings["Camera.height"] ;
    img_width = fsSettings["Camera.width"] ;
    renderingK = fsSettings["renderingK"] ;

    int displayResult = fsSettings["displayDelaunyResult"] ;
    displayDelaunyResult = (displayResult>0) ;
    int displayCenter = fsSettings["displayCenterFrame"] ;
    displayCenterFrame = (displayCenter>0);
    int displayUnOptimizedPointsFlag = fsSettings["displayUnOptimizedPoints"] ;
    displayUnOptimizedPoints = (displayUnOptimizedPointsFlag>0) ;
    int displayV = fsSettings["displayViewer"] ;
    displayViewer = displayV>0 ;


    meshSubdivNum = fsSettings["meshSubdivNum"] ;
    cvWaitTime = fsSettings["cvWaitTime"] ;
    pointQualityThreshold = fsSettings["pointQualityThreshold"] ;
    pointUpdateThreshold = fsSettings["pointUpdateThreshold"] ;
    constraintUpdateThreshold = fsSettings["constraintUpdateThreshold"] ;
    distanceThreshold = fsSettings["distanceThreshold"] ;

    colorMap = FColorMap(64) ;

    outlierVertexList.clear();
    points.clear() ;
    tris.clear();

    updatedLatestKFID = -1 ;
    curImgTime = -1 ;
    sumSpendTime = 0;
    sumKF = 0;
}

LocalMeshing::~LocalMeshing()
{
    ;
}

void LocalMeshing::computePubPointAndColor( float x, float y, float z, const Eigen::Vector3d& currentT,
                                            geometry_msgs::Point& p, std_msgs::ColorRGBA& color_p )
{
    p.x = x ;
    p.y = y ;
    p.z = z ;

    Eigen::Vector3d tp ;
    tp << x, y, z ;

    Eigen::Vector3d t0 = tp-currentT ;
    double d0 = t0.norm() ;
    if ( d0 > 10 ){
        d0 = 10.0 ;
    }
    int i0 = d0/10*62 ;
    cv::Vec3b col = colorMap.at(i0) ;
    color_p.a = 1.0 ;
    color_p.r = col[0]/255.0 ;
    color_p.g = col[1]/255.0 ;
    color_p.b = col[2]/255.0 ;
}

void LocalMeshing::updatePubPointList( const cv::Point3f& p3, const Eigen::Vector3d& currentT,
                                       std::vector<geometry_msgs::Point>& pointList,
                                       std::vector<std_msgs::ColorRGBA>& colorList )
{
    geometry_msgs::Point p ;
    p.x = p3.x ;
    p.y = p3.y ;
    p.z = p3.z ;
    pointList.push_back(p);

    Eigen::Vector3d tp ;
    tp << p3.x, p3.y, p3.z ;

    Eigen::Vector3d t0 = tp-currentT ;
    double d0 = t0.norm() ;
    if ( d0 > 10 ){
        d0 = 10.0 ;
    }
    int i0 = d0/10*62 ;
    cv::Vec3b col = colorMap.at(i0) ;
    std_msgs::ColorRGBA color_p ;
    color_p.a = 1.0 ;
    color_p.r = col[0]/255.0 ;
    color_p.g = col[1]/255.0 ;
    color_p.b = col[2]/255.0 ;
    colorList.push_back(color_p);
}

bool LocalMeshing::reconstructFrameMesh(KeyFrame *mpCurrentKF,
                                        std::map<std::tuple<float, float, float>, KeyFrame*>& mapPointCorrelation, int frameNum, Eigen::Vector3d centerT)
{
    //    if ( mpCurrentKF == NULL ){
    //        return false ;
    //    }
    //    if ( mpCurrentKF->img.data == NULL || mpCurrentKF->isBad() ){
    //        return false;
    //    }
    //    {
    //        unique_lock<mutex> lock(mpCurrentKF->mMutexOpertaion);
    //        if ( mpCurrentKF->isBad() ){
    //            return false ;
    //        }
    //        mpCurrentKF->SetNotErase();
    //    }

    //    double fx = mpCurrentKF->fx;
    //    double fy = mpCurrentKF->fy;
    //    double cx = mpCurrentKF->cx;
    //    double cy = mpCurrentKF->cy;

    //    cv::Mat currentPose = mpCurrentKF->GetPoseInverse();
    //    Eigen::Matrix3d currentR ;
    //    Eigen::Vector3d currentT(currentPose.at<float>(0,3), currentPose.at<float>(1,3), currentPose.at<float>(2,3)) ;
    //    currentR << currentPose.at<float>(0,0), currentPose.at<float>(0,1), currentPose.at<float>(0,2),
    //            currentPose.at<float>(1,0), currentPose.at<float>(1,1), currentPose.at<float>(1,2),
    //            currentPose.at<float>(2,0), currentPose.at<float>(2,1), currentPose.at<float>(2,2);

    //    const vector<MapPoint*>& vpMapPoints = mpCurrentKF->GetMapPointMatches();
    //    int currentKFID = mpCurrentKF->mnId ;

    //    CONVEX_HULL convex_hull ;
    //    convex_hull.pList.clear();
    //    std::map<GEOM_FADE2D::Point2, cv::Point3f> convexHullMap ;
    //    for( auto iter: mapPointCorrelation )
    //    {
    //        std::tuple<float, float, float> p3 = iter.first ;
    //        Eigen::Vector3d p3_w , p3_c ;
    //        std::get<0>(p3) ;
    //        p3_w << std::get<0>(p3), std::get<1>(p3), std::get<2>(p3) ;
    //        p3_c = currentR.transpose()*(p3_w - currentT) ;

    //        if ( p3_c(2) < 0.1 ){
    //            continue ;
    //        }

    //        float _u = p3_c(0)/p3_c(2)*fx+cx ;
    //        float _v = p3_c(1)/p3_c(2)*fy+cy ;

    //        convex_hull.insert(_u, _v);

    //        GEOM_FADE2D::Point2 p2(_u, _v) ;
    //        cv::Point3f p3_cv(p3_w(0), p3_w(1), p3_w(2)) ;
    //        convexHullMap.insert( std::pair<GEOM_FADE2D::Point2, cv::Point3f>(p2, p3_cv) ) ;
    //    }
    //    if ( mapPointCorrelation.size() > 3 ){
    //        convex_hull.graham_scan();
    //    }

    //    for( int ithNum = 0 ; ithNum <= meshSubdivNum ; ithNum++ )
    //    {
    //        cv::Mat curImg ;
    //        if ( displayDelaunyResult || (displayCenterFrame && (frameNum < 0)) ){
    //            cv::cvtColor(mpCurrentKF->img, curImg, CV_GRAY2BGR ) ;
    //        }

    //        //add the constriant in Delaunay construction
    //        GEOM_FADE2D::Fade_2D dt ;
    //        std::map<GEOM_FADE2D::Point2, cv::Point3f> kpMap ;
    //        std::map<GEOM_FADE2D::Point2, cv::Point3f>::iterator kpIt ;
    //        std::vector<GEOM_FADE2D::Segment2> vSegments;
    //        for( int i = 0, sz = convex_hull.corners.size() ; i < sz ; i++ )
    //        {
    //            int j = (i+1)%sz ;
    //            GEOM_FADE2D::Point2 p2_i(convex_hull.corners[i].x, convex_hull.corners[i].y) ;
    //            GEOM_FADE2D::Point2 p2_j(convex_hull.corners[j].x, convex_hull.corners[j].y) ;

    //            dt.insert(p2_i) ;
    //            vSegments.push_back( GEOM_FADE2D::Segment2(p2_i, p2_j) );

    //            if ( displayDelaunyResult )
    //            {
    //                cv::line(curImg, cv::Point2f(p2_i.x(), p2_i.y()),
    //                         cv::Point2f(p2_j.x(), p2_j.y()), cv::Scalar(0, 255, 0), 6,
    //                         CV_AA);
    //            }

    //            std::map<GEOM_FADE2D::Point2, cv::Point3f>::iterator it =
    //                    convexHullMap.find(p2_i) ;
    //            kpMap.insert( std::pair<GEOM_FADE2D::Point2, cv::Point3f>(p2_i, it->second) ) ;
    //        }

    //        //current view points
    //        for( int i=0, sz = vpMapPoints.size() ; i < sz ; i++ )
    //        {
    //            MapPoint* pMP = vpMapPoints[i] ;
    //            if ( pMP == NULL || pMP->isBad() )//Pruple
    //            {
    //                if ( displayDelaunyResult || (displayCenterFrame && (frameNum < 0) ) )
    //                {
    //                    if ( displayUnOptimizedPoints ){
    //                        cv::circle(curImg, mpCurrentKF->mvKeysUn[i].pt,
    //                                   0.2*mpCurrentKF->mvKeysUn[i].size, cv::Scalar(250, 20, 200), 1) ;
    //                    }
    //                }
    //                continue ;
    //            }

    //            //            if ( pMP->quality < 0 )//light blue
    //            //            {
    //            //                if ( displayDelaunyResult || (displayCenterFrame && (frameNum < 0) ) )
    //            //                {
    //            //                    if ( displayUnOptimizedPoints ){
    //            //                        cv::circle(curImg, mpCurrentKF->mvKeysUn[i].pt,
    //            //                                   0.2*mpCurrentKF->mvKeysUn[i].size, cv::Scalar(250, 200, 20), 1) ;
    //            //                    }
    //            //                }
    //            //                continue ;
    //            //            }

    //            cv::Mat Xw = pMP->GetWorldPos();
    //            Eigen::Vector3d p3_, p3_c ;
    //            p3_ << Xw.at<float>(0), Xw.at<float>(1), Xw.at<float>(2) ;

    //            p3_c = currentR.transpose()*(p3_ - currentT) ;
    //            if ( fabs(p3_c(2) -
    //                      mpCurrentKF->depth.at<float>(mpCurrentKF->mvKeysUn[i].pt.y, mpCurrentKF->mvKeysUn[i].pt.x)) > 0.1 )
    //            {
    //                if ( displayDelaunyResult || (displayCenterFrame && (frameNum < 0) ) )
    //                {
    //                    if ( displayUnOptimizedPoints ){//light blue
    //                        cv::circle(curImg, mpCurrentKF->mvKeysUn[i].pt,
    //                                   0.2*mpCurrentKF->mvKeysUn[i].size, cv::Scalar(250, 200, 20), 1) ;
    //                    }
    //                }
    //                continue ;
    //            }

    //            //            if ( (p3_-currentT).norm() < 1.0 )
    //            //            {
    //            ////                printf("quality= %f\n", pMP->quality ) ;
    //            ////                cv::circle(curImg, cv::Point2f(mpCurrentKF->mvKeysUn[i].pt.x, mpCurrentKF->mvKeysUn[i].pt.y),
    //            ////                           5, cv::Scalar(0, 255, 0), 5) ;
    //            //                continue ;
    //            //            }
    //            cv::Point2f tt = mpCurrentKF->mvKeysUn[i].pt ;
    //            float size = mpCurrentKF->mvKeysUn[i].size ;
    //            if ( tt.x >= img_width || tt.x < 0
    //                 || tt.y >= img_height || tt.y < 0 ){
    //                continue ;
    //            }

    //            if ( convex_hull.insideHull(tt.x, tt.y) ){
    //                continue ;
    //            }

    //            if ( displayDelaunyResult || (displayCenterFrame && (frameNum < 0) ) ){//Green
    //                cv::circle(curImg, tt, 0.2*size, cv::Scalar(20, 250, 0), 1) ;
    //            }

    //            GEOM_FADE2D::Point2 p2(tt.x, tt.y) ;
    //            cv::Point3f p3(Xw.at<float>(0), Xw.at<float>(1), Xw.at<float>(2)) ;
    //            kpMap.insert( std::pair<GEOM_FADE2D::Point2, cv::Point3f>(p2, p3) ) ;
    //            dt.insert(p2) ;
    //        }

    //        //        //current assistPoints
    //        //        for( int i=0, sz = mpCurrentKF->assistPointList.size() ; i < sz ; i++ )
    //        //        {
    //        //            GEOM_FADE2D::Point2 p2(mpCurrentKF->assistPointList[i].u, mpCurrentKF->assistPointList[i].v) ;

    //        //            if ( displayDelaunyResult || (displayCenterFrame && (frameNum < 0)) ){
    //        //                cv::Vec3b col = colorMap.at(ithNum) ;
    //        ////                cv::circle(curImg, cv::Point2f(p2.x(), p2.y()), 2,
    //        ////                           cv::Scalar(col.val[0], col.val[1], col.val[2]), 2) ;
    //        //                cv::circle(curImg, cv::Point2f(p2.x(), p2.y()), 2,
    //        //                           cv::Scalar(255, 255, 0), 2) ;
    //        //            }


    //        //            if ( convex_hull.insideHull(mpCurrentKF->assistPointList[i].u, mpCurrentKF->assistPointList[i].v) ){
    //        //                continue ;
    //        //            }

    //        //            Eigen::Vector3d p3d_ ;
    //        //            p3d_ << (mpCurrentKF->assistPointList[i].u-cx)/fx, (mpCurrentKF->assistPointList[i].v-cy)/fy, 1.0 ;
    //        //            p3d_ *= mpCurrentKF->assistPointList[i].depth ;
    //        //            Eigen::Vector3d p3d_2 = currentR*p3d_ + currentT ;
    //        //            cv::Point3f p3(p3d_2(0), p3d_2(1), p3d_2(2)) ;
    //        //            kpMap.insert( std::pair<GEOM_FADE2D::Point2, cv::Point3f>(p2, p3) ) ;
    //        //            dt.insert(p2) ;
    //        //        }

    //        if ( vSegments.size() > 0 )
    //        {
    //            dt.createConstraint(vSegments, GEOM_FADE2D::CIS_CONSTRAINED_DELAUNAY);
    //            dt.applyConstraintsAndZones();
    //        }

    //        //after triangulation
    //        std::vector<GEOM_FADE2D::Triangle2*> vAllDelaunayTriangles;
    //        dt.getTrianglePointers(vAllDelaunayTriangles);
    //        for(std::vector<GEOM_FADE2D::Triangle2*>::iterator it=vAllDelaunayTriangles.begin();
    //            it!=vAllDelaunayTriangles.end(); ++it )
    //        {
    //            GEOM_FADE2D::Triangle2* pT(*it);
    //            Eigen::Matrix3d A ;
    //            Eigen::Vector3d b ;
    //            int lx = 10000 ;
    //            int rx = -10000 ;
    //            int ly = 10000 ;
    //            int ry = -10000 ;
    //            Eigen::Vector3d p0_3d ;
    //            Eigen::Vector3d p1_3d ;
    //            Eigen::Vector3d p2_3d ;

    //            std::vector<geometry_msgs::Point> pointList ;
    //            std::vector<std_msgs::ColorRGBA> colorList ;
    //            cv::Point3f point3fList[3] ;

    //            bool flagOutside = false ;

    //            {//point 0
    //                GEOM_FADE2D::Point2 p2 = *pT->getCorner(0) ;
    //                if ( convex_hull.insideHull(p2.x(), p2.y()) == false ){
    //                    flagOutside = true ;
    //                }
    //                updateXYbound(p2.x(), p2.y(), lx, rx, ly, ry ) ;

    //                kpIt = kpMap.find(p2) ;
    //                if ( kpIt == kpMap.end() ){
    //                    //goto DISPLAY ;
    //                    continue ;
    //                }
    //                cv::Point3f p3 = kpIt->second ;
    //                point3fList[0] = p3 ;
    //                updatePubPointList(p3, centerT, pointList, colorList ) ;
    //                p0_3d << p3.x, p3.y, p3.z ;
    //            }
    //            {//point 1
    //                GEOM_FADE2D::Point2 p2 = *pT->getCorner(1) ;
    //                if ( convex_hull.insideHull(p2.x(), p2.y()) == false ){
    //                    flagOutside = true ;
    //                }
    //                updateXYbound(p2.x(), p2.y(), lx, rx, ly, ry ) ;

    //                kpIt = kpMap.find(p2) ;
    //                if ( kpIt == kpMap.end() ){
    //                    //goto DISPLAY ;
    //                    continue ;
    //                }
    //                cv::Point3f p3 = kpIt->second ;
    //                point3fList[1] = p3 ;
    //                updatePubPointList(p3, centerT, pointList, colorList ) ;
    //                p1_3d << p3.x, p3.y, p3.z ;
    //            }
    //            {//point 2
    //                GEOM_FADE2D::Point2 p2 = *pT->getCorner(2) ;
    //                if ( convex_hull.insideHull(p2.x(), p2.y()) == false ){
    //                    flagOutside = true ;
    //                }
    //                updateXYbound(p2.x(), p2.y(), lx, rx, ly, ry ) ;

    //                kpIt = kpMap.find(p2) ;
    //                if ( kpIt == kpMap.end() ){
    //                    //goto DISPLAY ;
    //                    continue ;
    //                }
    //                cv::Point3f p3 = kpIt->second ;
    //                point3fList[2] = p3 ;
    //                updatePubPointList(p3, centerT, pointList, colorList ) ;
    //                p2_3d << p3.x, p3.y, p3.z ;
    //            }

    //            if ( flagOutside == false ){
    //                continue ;
    //            }

    //            if ( ithNum == meshSubdivNum )
    //            {
    //                //check and update mapPointCorrelation
    //                for( int ii = 0 ; ii < 3 ; ii++ )
    //                {
    //                    std::tuple<float, float, float> p3(point3fList[ii].x, point3fList[ii].y, point3fList[ii].z);
    //                    std::map<std::tuple<float, float, float>, KeyFrame*>::iterator it
    //                            = mapPointCorrelation.find(p3) ;
    //                    if ( it == mapPointCorrelation.end() ){
    //                        mapPointCorrelation.insert( std::make_pair(p3, mpCurrentKF) ) ;
    //                    }
    //                }

    //                meshMsg_.points.push_back(pointList[0]);
    //                meshMsg_.points.push_back(pointList[1]);
    //                meshMsg_.points.push_back(pointList[1]);
    //                meshMsg_.points.push_back(pointList[2]);
    //                meshMsg_.points.push_back(pointList[2]);
    //                meshMsg_.points.push_back(pointList[0]);

    //                meshMsg_.colors.push_back(colorList[0]);
    //                meshMsg_.colors.push_back(colorList[1]);
    //                meshMsg_.colors.push_back(colorList[1]);
    //                meshMsg_.colors.push_back(colorList[2]);
    //                meshMsg_.colors.push_back(colorList[2]);
    //                meshMsg_.colors.push_back(colorList[0]);
    //            }

    //            if ( ithNum != meshSubdivNum && frameNum < 0 )
    //            {
    //                GEOM_FADE2D::Point2 pA = *pT->getCorner(0) ;
    //                GEOM_FADE2D::Point2 pB = *pT->getCorner(1) ;
    //                GEOM_FADE2D::Point2 pC = *pT->getCorner(2) ;

    //                myVector2d AB( pB.x()-pA.x(), pB.y()-pA.y() ) ;
    //                myVector2d AC( pC.x()-pA.x(), pC.y()-pA.y() ) ;
    //                double area = 0.5* AB.CrossProduct(AC) ;
    //                //printf("area = %lf\n", area ) ;
    //                //printf("lx=%d rx=%d ly=%d ry=%d\n", lx, rx, ly, ry ) ;
    //                if ( area < 3000 ){
    //                    ;
    //                }
    //                else
    //                {
    //                    p0_3d = currentR.transpose()*(p0_3d - currentT) ;
    //                    p1_3d = currentR.transpose()*(p1_3d - currentT) ;
    //                    p2_3d = currentR.transpose()*(p2_3d - currentT) ;
    //                    A.row(0) = p0_3d.transpose();
    //                    A.row(1) = p1_3d.transpose();
    //                    A.row(2) = p2_3d.transpose();
    //                    b << -1.0, -1.0, -1.0;
    //                    Eigen::Vector3d normal = A.colPivHouseholderQr().solve(b);
    //                    normal /= normal.norm() ;
    //                    double distance = -1.0/3.0*( (p0_3d.transpose()*normal)(0, 0)+
    //                                                 (p1_3d.transpose()*normal)(0, 0)+
    //                                                 (p2_3d.transpose()*normal)(0, 0)) ;
    //                    //            std::cout << "p0_3d = " << p0_3d.transpose() << " " ;
    //                    //            std::cout << (p0_3d.transpose()*normal)(0, 0)+distance << "\n" ;
    //                    //            std::cout << "p1_3d = " << p1_3d.transpose() << " " << (p1_3d.transpose()*normal)(0, 0)+distance << "\n" ;
    //                    //            std::cout << "p2_3d = " << p2_3d.transpose() << " " << (p2_3d.transpose()*normal)(0, 0)+distance << "\n" ;

    //                    GEOM_FADE2D::Point2 p0 = *pT->getCorner(0) ;
    //                    GEOM_FADE2D::Point2 p1 = *pT->getCorner(1) ;
    //                    GEOM_FADE2D::Point2 p2 = *pT->getCorner(2) ;

    //                    double max_diff = -1.0 ;
    //                    int max_diff_u = 0;
    //                    int max_diff_v = 0;
    //                    for( int ty = ly; ty <= ry ; ty++ )
    //                    {
    //                        for( int tx = lx; tx <= rx; tx++ )
    //                        {
    //                            if ( tx < 4 || tx >= img_width-4 || ty < 4 || ty >= img_height-4 ){
    //                                continue ;
    //                            }
    //                            myVector2d PA( double(tx)-p0.x(), double(ty)-p0.y() ) ;
    //                            myVector2d PB( double(tx)-p1.x(), double(ty)-p1.y() ) ;
    //                            myVector2d PC( double(tx)-p2.x(), double(ty)-p2.y() ) ;
    //                            if ( IsPointInTriangle(PA, PB, PC) == false ){
    //                                continue ;
    //                            }
    //                            if( mpCurrentKF->depth.at<float>(ty, tx) < 4.5 ) {
    //                                continue ;
    //                            }
    //                            //curImg.at<cv::Vec3b>(ty, tx) = cv::Vec3b(0, 255, 0) ;
    //                            Eigen::Vector3d v ;
    //                            v << (tx-cx)/fx, (ty-cy)/fy, 1.0 ;
    //                            double lamdba = -distance/(normal.transpose()*v)(0, 0) ;
    //                            //v *= lamdba ;
    //                            //printf("depth=%f lamdba=%f\n", mpCurrentKF->depth.at<float>(ty, tx), lamdba ) ;
    //                            double depth_difference = fabs(mpCurrentKF->depth.at<float>(ty, tx) - lamdba) ;
    //                            if ( depth_difference > max_diff ){
    //                                max_diff = depth_difference ;
    //                                max_diff_u = tx ;
    //                                max_diff_v = ty ;
    //                            }
    //                        }
    //                    }
    //                    if ( max_diff > 0.05 )
    //                    {
    //                        attachPoints tmp ;
    //                        tmp.depth = mpCurrentKF->depth.at<float>(max_diff_v, max_diff_u) ;
    //                        tmp.u = max_diff_u ;
    //                        tmp.v = max_diff_v ;
    //                        mpCurrentKF->assistPointList.push_back(tmp);
    //                    }
    //                }
    //            }//end of subdivision

    //            if ( displayDelaunyResult || (displayCenterFrame && (frameNum < 0) ) )
    //            {
    //                GEOM_FADE2D::Point2 p0 = *pT->getCorner(0) ;
    //                GEOM_FADE2D::Point2 p1 = *pT->getCorner(1) ;
    //                GEOM_FADE2D::Point2 p2 = *pT->getCorner(2) ;
    //                //                cv::circle(curImg, cv::Point2f(p0.x(), p0.y()), 2, cv::Scalar(255, 0, 0), 2) ;
    //                //                cv::circle(curImg, cv::Point2f(p1.x(), p1.y()), 2, cv::Scalar(255, 0, 0), 2) ;
    //                //                cv::circle(curImg, cv::Point2f(p2.x(), p2.y()), 2, cv::Scalar(255, 0, 0), 2) ;

    //                cv::line(curImg, cv::Point2f(p0.x(), p0.y()),
    //                         cv::Point2f(p1.x(), p1.y()), cv::Scalar(0, 0, 255), 1,
    //                         CV_AA);
    //                cv::line(curImg, cv::Point2f(p2.x(), p2.y()),
    //                         cv::Point2f(p1.x(), p1.y()), cv::Scalar(0, 0, 255), 1,
    //                         CV_AA);
    //                cv::line(curImg, cv::Point2f(p0.x(), p0.y()),
    //                         cv::Point2f(p2.x(), p2.y()), cv::Scalar(0, 0, 255), 1,
    //                         CV_AA);
    //            }
    //        }

    //        if ( ithNum == meshSubdivNum )
    //        {
    //            if ( displayDelaunyResult || (displayCenterFrame && (frameNum < 0) ) )
    //            {
    //                //                if ( frameNum < 0 )
    //                //                {
    //                //                    int baseline=0;

    //                //                    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    //                //                    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    //                //                    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    //                //                    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    //                //                    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);
    //                //                }
    //                if ( curImg.rows != 0 && curImg.cols != 0 ){
    //                    cv::imshow(std::to_string(frameNum), curImg ) ;

    //                    cv::Mat disp8;
    //                    cv::normalize(mpCurrentKF->depth, disp8, 0, 255, CV_MINMAX, CV_8U);
    //                    cv::imshow(std::string("depth")+ std::to_string(frameNum), disp8 ) ;
    //                }

    //                onView = true ;
    //                cv::waitKey(0) ;
    //                onView = false ;
    //            }
    //        }
    //    }
    //    mpCurrentKF->SetErase();

    //    return true ;
}

void LocalMeshing::buildMesh()
{
    KeyFrame* mpCurrentKF;
    {
        unique_lock<mutex> lock(mMutexKeyFrameQueue);
        if ( mlpKeyFrameQueue.size() < 3 ){
            return ;
        }
        mpCurrentKF = mlpKeyFrameQueue.front();
        mlpKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }
    if ( mpCurrentKF == NULL ){
        return ;
    }
    if ( mpCurrentKF->img.data == NULL || mpCurrentKF->isBad() ){
        return ;
    }

    std::map<std::tuple<float, float, float>, KeyFrame*> mapPointCorrelation;
    ros::Time curTime = ros::Time::now() ;

    meshMsg_.header.frame_id = "world";
    meshMsg_.header.stamp = curTime ;
    meshMsg_.type = visualization_msgs::Marker::LINE_LIST;
    //  meshMsg_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    meshMsg_.action = visualization_msgs::Marker::ADD ;
    //    meshMsg_.scale.x = 1.0 ;
    //    meshMsg_.scale.y = 1.0 ;
    //    meshMsg_.scale.z = 1.0 ;
    meshMsg_.scale.x = 0.01 ;
    meshMsg_.scale.y = 0.01 ;
    meshMsg_.scale.z = 0.01 ;

    meshMsg_.color.a = 1.0; // Don't forget to set the alpha!
    meshMsg_.color.r = 1.0;
    meshMsg_.color.g = 1.0;
    meshMsg_.color.b = 1.0;
    meshMsg_.points.clear();
    meshMsg_.colors.clear();

    cv::Mat currentPose = mpCurrentKF->GetPoseInverse();
    Eigen::Matrix3d currentR ;
    Eigen::Vector3d currentT(currentPose.at<float>(0,3), currentPose.at<float>(1,3), currentPose.at<float>(2,3)) ;
    currentR << currentPose.at<float>(0,0), currentPose.at<float>(0,1), currentPose.at<float>(0,2),
            currentPose.at<float>(1,0), currentPose.at<float>(1,1), currentPose.at<float>(1,2),
            currentPose.at<float>(2,0), currentPose.at<float>(2,1), currentPose.at<float>(2,2);
    mpCurrentKF->assistPointList.clear();
    reconstructFrameMesh(mpCurrentKF, mapPointCorrelation, -1, currentT) ;
    vector<KeyFrame*> neighboringKFs = mpCurrentKF->GetVectorCovisibleKeyFrames() ;
    //vector<int> neighboringKFWeights = mpCurrentKF->GetVectorCovisibleKeyFramesWeights();

    int neighbouringKFsz = neighboringKFs.size() ;
    if ( (int)neighboringKFs.size() < renderingK )
    {
        for(int i=0; i < neighbouringKFsz; i++ ){
            reconstructFrameMesh( neighboringKFs[i], mapPointCorrelation, i, currentT) ;
        }
    }
    else
    {
        for(int i=neighboringKFs.size()-renderingK; i < neighbouringKFsz; i++ ){
            reconstructFrameMesh( neighboringKFs[i], mapPointCorrelation, i-neighbouringKFsz+renderingK, currentT ) ;
        }
    }

    //    //to do, find the neighbouring keyframes
    //    int currentKFID = mpCurrentKF->mnId ;

    //    //already sorted from large to small

    //    std::vector<Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d> > Rwc(neighbouringKFsz) ;
    //    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > Twc(neighbouringKFsz) ;
    //    for(int i=0; i < neighbouringKFsz; i++ )
    //    {
    //        cv::Mat kFPose = neighboringKFs[i]->GetPoseInverse();
    //        Rwc[i] << kFPose.at<float>(0,0), kFPose.at<float>(0,1), kFPose.at<float>(0,2),
    //                kFPose.at<float>(1,0), kFPose.at<float>(1,1), kFPose.at<float>(1,2),
    //                kFPose.at<float>(2,0), kFPose.at<float>(2,1), kFPose.at<float>(2,2);
    //        Twc[i] << kFPose.at<float>(0,3), kFPose.at<float>(1,3), kFPose.at<float>(2,3);
    //    }


    pubMesh_.publish(meshMsg_) ;

    geometry_msgs::TransformStamped poseMsg_; //< Pose message.
    poseMsg_.header.frame_id = "world";
    poseMsg_.child_frame_id = "body";
    poseMsg_.header.stamp = curTime;
    Eigen::Quaterniond q(currentR) ;
    poseMsg_.transform.rotation.x = q.x();
    poseMsg_.transform.rotation.y = q.y();
    poseMsg_.transform.rotation.z = q.z();
    poseMsg_.transform.rotation.w = q.w();
    Eigen::Vector3d r = currentT;
    poseMsg_.transform.translation.x = r[0];
    poseMsg_.transform.translation.y = r[1];
    poseMsg_.transform.translation.z = r[2];
    //pubTf_.publish(poseMsg_) ;
    pubTf_.sendTransform(poseMsg_);

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = curTime;
    pose.header.frame_id = "world";
    pose.pose.position.x = r[0];
    pose.pose.position.y = r[1];
    pose.pose.position.z = r[2];
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pubPose_.publish(pose) ;

    nav_msgs::Odometry odometryMsg_;
    odometryMsg_.header.stamp = curTime ;
    odometryMsg_.header.frame_id = "world";
    odometryMsg_.child_frame_id = "body";
    // fill orientation
    odometryMsg_.pose.pose.orientation.x = q.x();
    odometryMsg_.pose.pose.orientation.y = q.y();
    odometryMsg_.pose.pose.orientation.z = q.z();
    odometryMsg_.pose.pose.orientation.w = q.w();
    // fill position
    odometryMsg_.pose.pose.position.x = r[0];
    odometryMsg_.pose.pose.position.y = r[1];
    odometryMsg_.pose.pose.position.z = r[2];
    pubObometry_.publish(odometryMsg_) ;

    mpCurrentKF->SetErase();

    //    {
    //        unique_lock<mutex> lock(mMutexKeyFrameQueue);
    //        mlpKeyFrameQueue.pop_front();
    //        mlpKeyFrameQueue.pop_front();
    //    }
}

void LocalMeshing::findKnearestKF(KeyFrame* mpCurrentKF, list<KeyFrame*>& nearestKF )
{
    nearestKF.clear();

    std::set<KeyFrame*> vstkFMap ;
    std::list<KeyFrame*> Queue ;
    Queue.push_back(mpCurrentKF);
    vstkFMap.insert(mpCurrentKF);
    while ( Queue.size() > 0 )
    {
        KeyFrame* currentKF = Queue.front() ;
        Queue.pop_front();
        nearestKF.push_back(currentKF);
        if ( (int)nearestKF.size() >= renderingK ){
            //done
            return ;
        }
        vector<KeyFrame*> neighboringKFs = currentKF->GetVectorCovisibleKeyFrames() ;
        vector<int> neighboringKFWeights = currentKF->GetVectorCovisibleKeyFramesWeights();
        for( int i = 0, sz = neighboringKFs.size(); i < sz; i++ )
        {
            if ( neighboringKFs[i]->isBad() ){
                removePoseInDt(neighboringKFs[i]) ;
                continue ;
            }
            //            if ( neighboringKFWeights[i] < 30 ){
            //                break ;
            //            }
            if ( vstkFMap.find(neighboringKFs[i]) == vstkFMap.end() ){
                Queue.push_back(neighboringKFs[i]);
                vstkFMap.insert(neighboringKFs[i]) ;
            }
        }
    }
}

void LocalMeshing::removePoseInDt(KeyFrame* mpCurrentKF)
{
    list<int>currentList(mpCurrentKF->constraintIDList) ;
    for( list<int>::iterator iter = currentList.begin();
         iter != currentList.end(); iter++ )
    {
        int id = *iter ;
        m_pAlgorithm.removeConstraint(id);
    }
}

void LocalMeshing::checkChange(KeyFrame* mpCurrentKF)
{
    //    double t = cvGetTickCount() ;

    list<KeyFrame*> nearestKF ;
    findKnearestKF(mpCurrentKF, nearestKF) ;
    std::unordered_map<int, collectedPoint> pointsToupdate;

    //puts("begin check change") ;
    for( list<KeyFrame*>::iterator iter = nearestKF.begin();
         iter != nearestKF.end(); iter++ )
    {
        if ( (*iter)->isBad() ){
            removePoseInDt( *iter ) ;
            continue ;
        }
        const vector<MapPoint*> vpMapPoints = (*iter)->GetMapPointMatches();
        for( int i=0, sz = vpMapPoints.size() ; i < sz ; i++ )
        {
            MapPoint* pMP = vpMapPoints[i] ;
            if ( pMP == NULL || pMP->isBad() ){
                continue ;
            }
            if ( m_pAlgorithm.mapPointID2Info.find(pMP->mnId) == m_pAlgorithm.mapPointID2Info.end() ){
                //not in the 3d triangulation
                continue ;
            }
            if ( pointsToupdate.find(pMP->mnId) != pointsToupdate.end() ){
                //already in the list
                continue ;
            }
            cv::Mat Xw = pMP->GetWorldPos();
            collectedPoint tmp ;
            tmp.id = pMP->mnId ;
            tmp.x = Xw.at<float>(0);
            tmp.y = Xw.at<float>(1);
            tmp.z = Xw.at<float>(2);

            std::pair<int,collectedPoint> key(tmp.id, tmp);
            pointsToupdate.insert(key) ;
        }
    }

    //check point change
    list<int> moveVertexList ;
    for(auto it:pointsToupdate)
    {
        int vertexID = it.first ;
        auto pPIt = m_pAlgorithm.mapPointID2Info.find(vertexID);
        pPIt->second.p3d_new_x = it.second.x ;
        pPIt->second.p3d_new_y = it.second.y ;
        pPIt->second.p3d_new_z = it.second.z ;
        pPIt->second.check(pointUpdateThreshold) ;
        if ( pPIt->second.updateFlag ){
            //update vertex
            moveVertexList.push_back(vertexID);
            //m_pAlgorithm.moveVertex(vertexID);
        }
    }
    m_pAlgorithm.moveVertexSet(moveVertexList);

    //check pose change, update the constraint
    //puts("before check pose change") ;
    for( list<KeyFrame*>::iterator iter = nearestKF.begin();
         iter != nearestKF.end(); iter++ )
    {
        if ( (*iter)->isBad() ){
            removePoseInDt( *iter ) ;
            continue ;
        }
        cv::Mat currentPose = (*iter)->GetPoseInverse();
        float pose3d_new_x = currentPose.at<float>(0,3) ;
        float pose3d_new_y = currentPose.at<float>(1,3) ;
        float pose3d_new_z = currentPose.at<float>(2,3) ;

        list<int> currentList( (*iter)->constraintIDList ) ;
        for( auto constraintID: currentList )
        {
            auto pCIt = m_pAlgorithm.mapConstraintID2Info.find(constraintID) ;
            if ( pCIt == m_pAlgorithm.mapConstraintID2Info.end() )
            {
                puts("can not find constraint in checkChange") ;
                continue ;
            }
            //            pCIt->second.pose3d_new_x = pose3d_new_x ;
            //            pCIt->second.pose3d_new_y = pose3d_new_y ;
            //            pCIt->second.pose3d_new_z = pose3d_new_z ;

            int vertexID = pCIt->second.vertexID ;
            auto pPIt = m_pAlgorithm.mapPointID2Info.find(vertexID);
            if ( pPIt == m_pAlgorithm.mapPointID2Info.end() )
            {
                puts("can not find point in checkChange") ;
                continue ;
            }
            float p3d_new_x = pPIt->second.handle->point().x() ;
            float p3d_new_y = pPIt->second.handle->point().y() ;
            float p3d_new_z = pPIt->second.handle->point().z() ;

            float d1 = SQ(pose3d_new_x - pCIt->second.pose3d_old_x) +
                    SQ(pose3d_new_y - pCIt->second.pose3d_old_y) +
                    SQ(pose3d_new_z - pCIt->second.pose3d_old_z) ;
            d1 = sqrt(d1) ;

            float d2 = SQ(p3d_new_x - pCIt->second.v3d_old_x) +
                    SQ(p3d_new_y - pCIt->second.v3d_old_y) +
                    SQ(p3d_new_z - pCIt->second.v3d_old_z) ;
            d2 = sqrt(d2) ;
            if ( d1+d2 > constraintUpdateThreshold )
            {
                m_pAlgorithm.removeConstraint(constraintID);
                m_pAlgorithm.addConstraintKF(vertexID, (*iter) ) ;
            }
        }
    }
    //    puts("after check pose change") ;
    //    ROS_WARN("CheckChange Time %lf", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000));
}

void LocalMeshing::buildMesh2()
{
    KeyFrame* mpCurrentKF;
    {
        unique_lock<mutex> lock(mMutexKeyFrameQueue);
        if ( mlpKeyFrameQueue.size() < 1 ){
            return ;
        }
        mpCurrentKF = mlpKeyFrameQueue.front();
        mlpKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }
    if ( mpCurrentKF == NULL ){
        return ;
    }
    if ( mpCurrentKF->img.data == NULL || mpCurrentKF->isBad() ){
        return ;
    }

    double start_meshing = (double)cvGetTickCount();

    ros::Time curTime = ros::Time::now() ;

    cv::Mat currentPose = mpCurrentKF->GetPoseInverse();
    Eigen::Matrix3d currentR ;
    Eigen::Vector3d currentT(currentPose.at<float>(0,3), currentPose.at<float>(1,3), currentPose.at<float>(2,3)) ;
    currentR << currentPose.at<float>(0,0), currentPose.at<float>(0,1), currentPose.at<float>(0,2),
            currentPose.at<float>(1,0), currentPose.at<float>(1,1), currentPose.at<float>(1,2),
            currentPose.at<float>(2,0), currentPose.at<float>(2,1), currentPose.at<float>(2,2);
    mpCurrentKF->assistPointList.clear();

    const vector<MapPoint*>& vpMapPoints = mpCurrentKF->GetMapPointMatches();
    vector<KeyFrame*> neighboringKFs = mpCurrentKF->GetVectorCovisibleKeyFrames() ;
    //vector<int> neighboringKFWeights = mpCurrentKF->GetVectorCovisibleKeyFramesWeights();

    double t = cvGetTickCount() ;
    if ( displayDelaunyResult ){
        cv::cvtColor(mpCurrentKF->img, curImg, CV_GRAY2BGR ) ;
    }

    list<int>currentOutlierVertexList ;
    //1. delete the outlier points
    {
        unique_lock<mutex> lock(outlierVertexListQueue);
        currentOutlierVertexList = outlierVertexList ;
        outlierVertexList.clear();
    }
    for(list<int>::iterator iter = currentOutlierVertexList.begin();
        iter != currentOutlierVertexList.end();  )
    {
        auto it = m_pAlgorithm.mapPointID2Info.find( *iter ) ;
        if ( it == m_pAlgorithm.mapPointID2Info.end() ){
            iter = currentOutlierVertexList.erase(iter) ;
        }
        else{
            iter++ ;
        }
    }
    std::set<int> setUnionedConstraints ;
    m_pAlgorithm.removeVertexSet(currentOutlierVertexList, setUnionedConstraints);
    //    for( auto id: currentOutlierVertexList )
    //    {
    //        auto it = m_pAlgorithm.mapPointID2Info.find( id ) ;
    //        if ( it != m_pAlgorithm.mapPointID2Info.end() ){
    //            //printf("removeVertex: %d\n", id ) ;
    //            //m_pAlgorithm.removeVertex(id);
    //            m_pAlgorithm.removeVertex_origin(id) ;
    //            //puts(" done") ;
    //        }
    //    }

    //2. collect points or constraints
    updatedPointsAndConstraints.clear();
    for( int i=0, sz = vpMapPoints.size() ; i < sz ; i++ )
    {
        MapPoint* pMP = vpMapPoints[i] ;
        if ( pMP == NULL || pMP->isBad() )//Pruple
        {
            if ( displayDelaunyResult )
            {
                if ( displayUnOptimizedPoints ){
                    cv::circle(curImg, mpCurrentKF->mvKeysUn[i].pt,
                               0.2*mpCurrentKF->mvKeysUn[i].size, cv::Scalar(250, 20, 200), 1) ;
                }
            }
            continue ;
        }

        if ( pMP->quality < (double)pointQualityThreshold )//light blue
        {
            if ( displayDelaunyResult )
            {
                if ( displayUnOptimizedPoints ){
                    cv::circle(curImg, mpCurrentKF->mvKeysUn[i].pt,
                               0.2*mpCurrentKF->mvKeysUn[i].size, cv::Scalar(249, 230, 162), 1) ;
                }
            }
            continue ;
        }

        cv::Mat Xw = pMP->GetWorldPos();
        Eigen::Vector3d p3_, p3_c ;
        p3_ << Xw.at<float>(0), Xw.at<float>(1), Xw.at<float>(2) ;
        p3_c = currentR.transpose()*(p3_ - currentT) ;
        if ( fabs(p3_c(2) -
                  mpCurrentKF->depth.at<float>(mpCurrentKF->mvKeysUn[i].pt.y,
                                               mpCurrentKF->mvKeysUn[i].pt.x)) > distanceThreshold )
        {
            //printf("z=%lf depth=%f\n", p3_c(2), mpCurrentKF->depth.at<float>(mpCurrentKF->mvKeysUn[i].pt.y, mpCurrentKF->mvKeysUn[i].pt.x) ) ;
            if ( displayDelaunyResult )
            {
                if ( displayUnOptimizedPoints ){//orange
                    cv::circle(curImg, mpCurrentKF->mvKeysUn[i].pt,
                               0.2*mpCurrentKF->mvKeysUn[i].size, cv::Scalar(0, 20, 250), 1) ;
                }
            }
            continue ;
        }

        if ( displayDelaunyResult ){//Green
            cv::circle(curImg, mpCurrentKF->mvKeysUn[i].pt, 0.2*mpCurrentKF->mvKeysUn[i].size,
                       cv::Scalar(20, 250, 0), 1) ;
        }

        auto it = m_pAlgorithm.mapPointID2Info.find( pMP->mnId ) ;
        if ( it == m_pAlgorithm.mapPointID2Info.end() )//add point
        {
            collectedPoint tmp ;
            tmp.id = pMP->mnId ;
            tmp.x = Xw.at<float>(0) ;
            tmp.y = Xw.at<float>(1) ;
            tmp.z = Xw.at<float>(2) ;
            tmp.flag = true ;
            updatedPointsAndConstraints.push_back(tmp);
        }
        else//add constraint
        {
            collectedPoint tmp ;
            tmp.id = pMP->mnId ;
            tmp.flag = false ;
            updatedPointsAndConstraints.push_back(tmp);
        }
    }

    m_pAlgorithm.updateVertexAndConstraints(updatedPointsAndConstraints, mpCurrentKF);

    //3. add subdivision points
    //    for( int ithNum = 0 ; ithNum <= meshSubdivNum ; ithNum++ )
    //    {

    //    }

    //4. update points or constraints
    checkChange(mpCurrentKF) ;

    double end_meshing = (double)cvGetTickCount();
    sumSpendTime += (end_meshing-start_meshing)/ (cvGetTickFrequency() * 1000) ;
    sumKF++ ;
    printf("avergeSpendTime = %lf ms\n", sumSpendTime/sumKF ) ;

    mpCurrentKF->SetErase();
    mpCurrentKF->img.release();
    mpCurrentKF->depth.release();
    //printf("4 %lf\n", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000));

    updatedLatestKFID = mpCurrentKF->mnId ;

    //publish mesh
    if ( displayViewer || true )
    {
        unique_lock<mutex> lock(mMutexMesh);
        double t = (double)cvGetTickCount() ;
        m_pAlgorithm.tetsToTris_naive2(points, tris, edges, 1);
        //printf("2 %lf\n", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000));

        //printf("8 %d\n", m_pAlgorithm.dt.number_of_vertices() );
        //printf("9 %d\n", m_pAlgorithm.dt.number_of_cells() );
    }

    //    meshMsg_.header.frame_id = "world";
    //    meshMsg_.header.stamp = curTime ;
    //    meshMsg_.type = visualization_msgs::Marker::LINE_LIST;
    //    //  meshMsg_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    //    meshMsg_.action = visualization_msgs::Marker::ADD ;
    //    meshMsg_.scale.x = 0.01 ;
    //    meshMsg_.scale.y = 0.01 ;
    //    meshMsg_.scale.z = 0.01 ;
    //    meshMsg_.color.a = 1.0; // Don't forget to set the alpha!
    //    meshMsg_.color.r = 1.0;
    //    meshMsg_.color.g = 1.0;
    //    meshMsg_.color.b = 1.0;
    //    meshMsg_.points.clear();
    //    meshMsg_.colors.clear();
    //    for( int i=0, sz = tris.size(); i < sz; i++ )
    //    {
    //        geometry_msgs::Point pointList[3] ;
    //        std_msgs::ColorRGBA colorList[3] ;

    //        computePubPointAndColor(tris[i].p0.x, tris[i].p0.y, tris[i].p0.z, currentT, pointList[0], colorList[0]) ;
    //        computePubPointAndColor(tris[i].p1.x, tris[i].p1.y, tris[i].p1.z, currentT, pointList[1], colorList[1]) ;
    //        computePubPointAndColor(tris[i].p2.x, tris[i].p2.y, tris[i].p2.z, currentT, pointList[2], colorList[2]) ;

    //        meshMsg_.points.push_back(pointList[0]);
    //        meshMsg_.points.push_back(pointList[1]);
    //        meshMsg_.points.push_back(pointList[1]);
    //        meshMsg_.points.push_back(pointList[2]);
    //        meshMsg_.points.push_back(pointList[2]);
    //        meshMsg_.points.push_back(pointList[0]);

    //        meshMsg_.colors.push_back(colorList[0]);
    //        meshMsg_.colors.push_back(colorList[1]);
    //        meshMsg_.colors.push_back(colorList[1]);
    //        meshMsg_.colors.push_back(colorList[2]);
    //        meshMsg_.colors.push_back(colorList[2]);
    //        meshMsg_.colors.push_back(colorList[0]);
    //    }
    //    pubMesh_.publish(meshMsg_) ;

    //    //publish freespace
    //    m_pAlgorithm.tetsToFreeSpace(tris);
    //    freeSpaceMsg_.header.frame_id = "world";
    //    freeSpaceMsg_.header.stamp = ros::Time::now() ;
    //    freeSpaceMsg_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    //    freeSpaceMsg_.action = visualization_msgs::Marker::MODIFY ;
    //    freeSpaceMsg_.scale.x = 1 ;
    //    freeSpaceMsg_.scale.y = 1 ;
    //    freeSpaceMsg_.scale.z = 1 ;
    //    freeSpaceMsg_.color.a = 1.0; // Don't forget to set the alpha!
    //    freeSpaceMsg_.color.r = 1.0;
    //    freeSpaceMsg_.color.g = 1.0;
    //    freeSpaceMsg_.color.b = 1.0;
    //    freeSpaceMsg_.points.clear();
    //    freeSpaceMsg_.colors.clear();
    //    //printf("freeSpcaeSz=%d\n", tris.size() ) ;
    //    for( int i=0, sz = tris.size(); i < sz; i++ )
    //    {
    //        geometry_msgs::Point point;
    //        point.x = tris[i].p0.x ;
    //        point.y = tris[i].p0.y ;
    //        point.z = tris[i].p0.z ;
    //        freeSpaceMsg_.points.push_back(point);
    //        point.x = tris[i].p1.x ;
    //        point.y = tris[i].p1.y ;
    //        point.z = tris[i].p1.z ;
    //        freeSpaceMsg_.points.push_back(point);
    //        point.x = tris[i].p2.x ;
    //        point.y = tris[i].p2.y ;
    //        point.z = tris[i].p2.z ;
    //        freeSpaceMsg_.points.push_back(point);
    //    }
    //    pubFreeSpace_.publish(freeSpaceMsg_) ;

    //    geometry_msgs::TransformStamped poseMsg_; //< Pose message.
    //    poseMsg_.header.frame_id = "world";
    //    poseMsg_.child_frame_id = "body";
    //    poseMsg_.header.stamp = curTime;
    //    Eigen::Quaterniond q(currentR) ;
    //    poseMsg_.transform.rotation.x = q.x();
    //    poseMsg_.transform.rotation.y = q.y();
    //    poseMsg_.transform.rotation.z = q.z();
    //    poseMsg_.transform.rotation.w = q.w();
    //    Eigen::Vector3d r = currentT;
    //    poseMsg_.transform.translation.x = r[0];
    //    poseMsg_.transform.translation.y = r[1];
    //    poseMsg_.transform.translation.z = r[2];
    //    //pubTf_.publish(poseMsg_) ;
    //    pubTf_.sendTransform(poseMsg_);

    //    geometry_msgs::PoseStamped pose;
    //    pose.header.stamp = curTime;
    //    pose.header.frame_id = "world";
    //    pose.pose.position.x = r[0];
    //    pose.pose.position.y = r[1];
    //    pose.pose.position.z = r[2];
    //    pose.pose.orientation.x = q.x();
    //    pose.pose.orientation.y = q.y();
    //    pose.pose.orientation.z = q.z();
    //    pose.pose.orientation.w = q.w();
    //    pubPose_.publish(pose) ;

    //    nav_msgs::Odometry odometryMsg_;
    //    odometryMsg_.header.stamp = curTime ;
    //    odometryMsg_.header.frame_id = "world";
    //    odometryMsg_.child_frame_id = "body";
    //    // fill orientation
    //    odometryMsg_.pose.pose.orientation.x = q.x();
    //    odometryMsg_.pose.pose.orientation.y = q.y();
    //    odometryMsg_.pose.pose.orientation.z = q.z();
    //    odometryMsg_.pose.pose.orientation.w = q.w();
    //    // fill position
    //    odometryMsg_.pose.pose.position.x = r[0];
    //    odometryMsg_.pose.pose.position.y = r[1];
    //    odometryMsg_.pose.pose.position.z = r[2];
    //    pubObometry_.publish(odometryMsg_) ;

    if ( displayDelaunyResult )
    {
        onView = true ;
        cv::imshow("Current Keyframe", curImg) ;
        //cv::moveWindow("Current Keyframe", 0, 700) ;
        cv::waitKey(cvWaitTime) ;
        onView = false ;
    }

    {
        //        unique_lock<mutex> lock(mMutexKeyFrameQueue);
        //        while( mlpKeyFrameQueue.size() > 5 ){
        //            mlpKeyFrameQueue.pop_front();
        //        }
    }
}

void LocalMeshing::updateLocalVisibilityConstraints()
{
    double t = cvGetTickCount() ;
    for( int ith = 0; ith < 1 ; ith++ )
    {
        int num ;
        {
            unique_lock<mutex> lock(mMutexLocalFrameInfoQueue);
            num = mLocalFrameInfoQueue.front();

            if ( updatedLatestKFID < mLocalFrameInfoUnion[num].referenceKF->mnId ){
                break ;
            }
            mLocalFrameInfoQueue.pop_front();
        }
        cv::Mat delta_T = cv::Mat(4, 4, CV_32F, &mLocalFrameInfoUnion[num].delta_T ) ;
        cv::Mat currentPose = mLocalFrameInfoUnion[num].referenceKF->GetPoseInverse()*delta_T;
        float pose_x = currentPose.at<float>(0,3);
        float pose_y = currentPose.at<float>(1,3);
        float pose_z = currentPose.at<float>(2,3);
        float time = mLocalFrameInfoUnion[num].time ;
        for( int i = 0, sz = mLocalFrameInfoUnion[num].vertexID.size() ; i < sz ; i++ ){
            m_pAlgorithm.addConstraintNonKF(mLocalFrameInfoUnion[num].vertexID[i], pose_x, pose_y, pose_z, time );
        }

        int sz ;
        {
            unique_lock<mutex> lock(mMutexLocalFrameInfoQueue);
            sz = mLocalFrameInfoQueue.size();
            if ( sz == 0 ){
                break ;
            }
        }
    }
    printf("[updateLocal] time: %f\n", ((double)cvGetTickCount() - t) / (cvGetTickFrequency() * 1000));
}

void LocalMeshing::Run()
{
    puts("start run") ;
    mbFinished =false;

    while ( ros::ok() )
    {
        // Check if there are keyframes in the queue
        if( !mlpKeyFrameQueue.empty() ){
            buildMesh2();
        }

        // Check if there are localVisibility constraints in the queue
        int sz ;
        {
            unique_lock<mutex> lock(mMutexLocalFrameInfoQueue);
            sz = mLocalFrameInfoQueue.size();
        }
        if( sz > 0 ){
            //updateLocalVisibilityConstraints();
        }

        if( mbFinished ){
            break;
        }

        usleep(1000);
    }
    mbFinished = true ;
}

void LocalMeshing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexKeyFrameQueue);
    if(pKF->mnId!=0)
        mlpKeyFrameQueue.push_back(pKF);
}

FreespaceDelaunayAlgorithm::FreespaceDelaunayAlgorithm()
{
    reset() ;
}

void FreespaceDelaunayAlgorithm::reset()
{
    Delaunay3CellInfoID = 0 ;
    mapPointID2Info.clear();
    mapConstraintID2Info.clear();
    dt.clear();
    constraintNextID = 0 ;
    createBounds();
}

void FreespaceDelaunayAlgorithm::createBounds()
{
    // Note: The two values below worked with arbitrary precision arithmetic (though slowed down the execution),
    // but caused numerical difficulties in some computational geometry algorithms when using double-precision
    // const double nLargePosDouble = (double) numeric_limits<float>::max();
    // const double nLargeNegDouble = - (double) numeric_limits<float>::max();
    //    const double nLargePosDouble = getBoundsMax();
    //    const double nLargeNegDouble = getBoundsMin();
    double nLargePosDouble = 10000000 ;
    double nLargeNegDouble = -10000000 ;

    // Insert 8 points forming an axis-aligned bounding box around our real data points.  Comments of + or - on the following lines indicates the limits of x y z, resp.,
    // that the corresponding corner point is at.
    dt.insert(PointD3(nLargePosDouble, nLargePosDouble, nLargePosDouble)); // + + +
    dt.insert(PointD3(nLargePosDouble, nLargePosDouble, nLargeNegDouble)); // + + -
    dt.insert(PointD3(nLargePosDouble, nLargeNegDouble, nLargePosDouble)); // + - +
    dt.insert(PointD3(nLargePosDouble, nLargeNegDouble, nLargeNegDouble)); // + - -
    dt.insert(PointD3(nLargeNegDouble, nLargePosDouble, nLargePosDouble)); // - + +
    dt.insert(PointD3(nLargeNegDouble, nLargePosDouble, nLargeNegDouble)); // - + -
    dt.insert(PointD3(nLargeNegDouble, nLargeNegDouble, nLargePosDouble)); // - - +
    dt.insert(PointD3(nLargeNegDouble, nLargeNegDouble, nLargeNegDouble)); // - - -
}

void FreespaceDelaunayAlgorithm::addConstraintNonKF( int vertexID, float pose_x, float pose_y, float pose_z, float time )
{
    auto pPIt = mapPointID2Info.find(vertexID) ;
    if ( pPIt == mapPointID2Info.end() )
    {
        //puts("can not find point in addConstraintNonKF") ;
        return ;
    }
    Delaunay3::Vertex_handle handle = pPIt->second.handle;

    //initialize a new constraint
    constraintInfo currentConstraint ;
    currentConstraint.constraintID = constraintNextID++ ;
    currentConstraint.pose = NULL ;
    currentConstraint.time = time ;
    currentConstraint.kind = CON_NONKF ;
    currentConstraint.pose3d_old_x = pose_x ;
    currentConstraint.pose3d_old_y = pose_y ;
    currentConstraint.pose3d_old_z = pose_z ;
    currentConstraint.updateFlag = false ;
    currentConstraint.vertexID = vertexID ;
    currentConstraint.v3d_old_x = handle->point().x() ;
    currentConstraint.v3d_old_y = handle->point().y() ;
    currentConstraint.v3d_old_z = handle->point().z() ;
    currentConstraint.listCell.clear();

    std::pair<int,constraintInfo> keyConstraint(currentConstraint.constraintID, currentConstraint);
    mapConstraintID2Info.insert(keyConstraint) ;

    Segment QO = Segment(handle->point(),
                         PointD3(currentConstraint.pose3d_old_x, currentConstraint.pose3d_old_y, currentConstraint.pose3d_old_z));

    //attach constraint to vectex
    pPIt->second.constraintIDList.push_back(currentConstraint.constraintID);

    //attach constraint to tetrahedra
    markTetrahedraCrossingConstraintWithBookKeeping(handle, QO, currentConstraint.constraintID) ;
}

void FreespaceDelaunayAlgorithm::addConstraintKF(int vertexID, KeyFrame *kF)
{
    auto pPIt = mapPointID2Info.find(vertexID) ;
    if ( pPIt == mapPointID2Info.end() )
    {
        puts("can not find point in addConstraintKF") ;
        return ;
    }
    Delaunay3::Vertex_handle handle = pPIt->second.handle;

    //initialize a new constraint
    constraintInfo currentConstraint ;
    currentConstraint.constraintID = constraintNextID++ ;
    currentConstraint.pose = kF ;
    currentConstraint.time = (float)kF->mTimeStamp ;
    currentConstraint.kind = CON_KF ;
    cv::Mat currentPose = kF->GetPoseInverse();
    currentConstraint.pose3d_old_x = currentPose.at<float>(0,3) ;
    currentConstraint.pose3d_old_y = currentPose.at<float>(1,3) ;
    currentConstraint.pose3d_old_z = currentPose.at<float>(2,3) ;
    currentConstraint.updateFlag = false ;
    currentConstraint.vertexID = vertexID ;
    currentConstraint.v3d_old_x = handle->point().x() ;
    currentConstraint.v3d_old_y = handle->point().y() ;
    currentConstraint.v3d_old_z = handle->point().z() ;
    currentConstraint.listCell.clear();

    std::pair<int,constraintInfo> keyConstraint(currentConstraint.constraintID, currentConstraint);
    mapConstraintID2Info.insert(keyConstraint) ;

    //attach constraint to pose
    kF->constraintIDList.push_back(currentConstraint.constraintID);
    //printf("[Push] cID=%d to kFid=%d\n", currentConstraint.constraintID, kF->mnId ) ;

    Segment QO = Segment(handle->point(),
                         PointD3(currentConstraint.pose3d_old_x, currentConstraint.pose3d_old_y, currentConstraint.pose3d_old_z));

    //attach constraint to vectex
    pPIt->second.constraintIDList.push_back(currentConstraint.constraintID);

    //attach constraint to tetrahedra
    markTetrahedraCrossingConstraintWithBookKeeping(handle, QO, currentConstraint.constraintID) ;
}

void FreespaceDelaunayAlgorithm::addSetOfConstraints(set<int>& setUnionedConstraints )
{
    for( set<int>::iterator iter = setUnionedConstraints.begin();
         iter != setUnionedConstraints.end(); iter++ )
    {
        int constraintID = *iter ;
        auto pCIt = mapConstraintID2Info.find(constraintID) ;
        if ( pCIt == mapConstraintID2Info.end() )
        {
            //puts("can not find constraint") ;
            continue ;
        }
        if ( pCIt->second.kind == CON_KF && pCIt->second.pose->isBad() ){//the keyframe is deleted or not
            continue ;
        }
        int vertexID = pCIt->second.vertexID ;
        auto pPIt = mapPointID2Info.find(vertexID) ;
        if ( pPIt == mapPointID2Info.end() )
        {
            puts("can not find point") ;
            continue ;
        }
        Delaunay3::Vertex_handle handle = pPIt->second.handle;
        Segment QO = Segment(handle->point(),
                             PointD3(pCIt->second.pose3d_old_x,
                                     pCIt->second.pose3d_old_y,
                                     pCIt->second.pose3d_old_z));
        markTetrahedraCrossingConstraintWithBookKeeping(handle, QO, constraintID ) ;
    }
}


void FreespaceDelaunayAlgorithm::moveVertexSet( const list<int>& moveVertexList )
{
    std::set<int> setUnionedConstraints;

    struct moveInfo
    {
        int vertexID ;
        float x, y, z ;
        list<KeyFrame*> KFlist ;
        vector<float> NONKF_pose_x, NONKF_pose_y, NONKF_pose_z, NONKF_pose_time ;
    };

    list<moveInfo> moveList ;
    for( auto vertexID: moveVertexList )
    {
        auto pPIt = mapPointID2Info.find(vertexID) ;
        if ( pPIt == mapPointID2Info.end() )
        {
            puts("can not find point in removeVertex") ;
            return ;
        }

        moveInfo tmp ;
        tmp.vertexID = vertexID ;
        tmp.x = pPIt->second.p3d_new_x ;
        tmp.y = pPIt->second.p3d_new_y ;
        tmp.z = pPIt->second.p3d_new_z ;
        tmp.KFlist.clear();
        tmp.NONKF_pose_x.clear();
        tmp.NONKF_pose_y.clear();
        tmp.NONKF_pose_z.clear();
        tmp.NONKF_pose_time.clear();
        for( auto constrainID: pPIt->second.constraintIDList )
        {
            auto pCIt = mapConstraintID2Info.find(constrainID) ;
            if ( pCIt == mapConstraintID2Info.end() )
            {
                puts("can not find constraint") ;
                return ;
            }
            if ( pCIt->second.kind == CON_KF && !pCIt->second.pose->isBad() ){
                tmp.KFlist.push_back(pCIt->second.pose);
            }
            if ( pCIt->second.kind == CON_NONKF ){
                tmp.NONKF_pose_x.push_back( pCIt->second.pose3d_old_x );
                tmp.NONKF_pose_y.push_back( pCIt->second.pose3d_old_y );
                tmp.NONKF_pose_z.push_back( pCIt->second.pose3d_old_z );
                tmp.NONKF_pose_time.push_back( pCIt->second.time );
            }
        }
        moveList.push_back(tmp);
    }
    //remove vertex
    std::list<int> toRemove(moveVertexList) ;
    removeVertexSet(toRemove, setUnionedConstraints);

    //add vertex
    for( auto moveVertex: moveList ){
        addVertex( moveVertex.vertexID, moveVertex.x, moveVertex.y, moveVertex.z, setUnionedConstraints );
    }

    //add back the deleted constraints related to delete cells
    addSetOfConstraints(setUnionedConstraints);

    //add back the KF constrain related to vertex
    for( auto moveVertex: moveList )
    {
        for( auto kF: moveVertex.KFlist )
        {
            if ( kF->isBad() ){
                continue ;
            }
            addConstraintKF(moveVertex.vertexID, kF);
        }
    }

    //add back the NONKF constrain related to vertex
    for( auto moveVertex: moveList )
    {
        for( int sz = moveVertex.NONKF_pose_x.size(), i = 0 ; i < sz; i++ ){
            addConstraintNonKF(moveVertex.vertexID, moveVertex.NONKF_pose_x[i],
                               moveVertex.NONKF_pose_y[i], moveVertex.NONKF_pose_z[i],
                               moveVertex.NONKF_pose_time[i] );
        }
    }
}

void FreespaceDelaunayAlgorithm::moveVertex(int vertexID )
{
    auto pPIt = mapPointID2Info.find(vertexID) ;
    if ( pPIt == mapPointID2Info.end() )
    {
        puts("can not find point in removeVertex") ;
        return ;
    }
    float new_x = pPIt->second.p3d_new_x;
    float new_y = pPIt->second.p3d_new_y;
    float new_z = pPIt->second.p3d_new_z;

    // collect the constraints related to the vertex
    std::list<KeyFrame*> kFList ;
    std::vector<float> NONKF_pose_x, NONKF_pose_y, NONKF_pose_z, NONKF_pose_time ;
    for( auto constrainID: pPIt->second.constraintIDList )
    {
        auto pCIt = mapConstraintID2Info.find(constrainID) ;
        if ( pCIt == mapConstraintID2Info.end() )
        {
            puts("can not find constraint") ;
            return ;
        }
        if ( pCIt->second.kind == CON_KF && !pCIt->second.pose->isBad() ){
            kFList.push_back(pCIt->second.pose);
        }
        if ( pCIt->second.kind == CON_NONKF){
            NONKF_pose_x.push_back( pCIt->second.pose3d_old_x );
            NONKF_pose_y.push_back( pCIt->second.pose3d_old_y );
            NONKF_pose_z.push_back( pCIt->second.pose3d_old_z );
            NONKF_pose_time.push_back( pCIt->second.time );
        }
    }

    //remove vertex
    removeVertex_origin(vertexID);

    //add vertex
    set<int> setUnionedConstraints;
    addVertex(vertexID, new_x, new_y, new_z, setUnionedConstraints );

    //add back the KF constrain related to vertex
    for( auto kF: kFList )
    {
        if ( kF->isBad() ){
            continue ;
        }
        addConstraintKF(vertexID, kF);
    }

    //add back the NONKF constrain related to vertex
    for( int sz = NONKF_pose_x.size(), i = 0 ; i < sz; i++ ){
        addConstraintNonKF(vertexID, NONKF_pose_x[i], NONKF_pose_y[i], NONKF_pose_z[i], NONKF_pose_time[i] );
    }

    //add back the deleted constraints related to delete cells
    for (set<int>::iterator itConstraint = setUnionedConstraints.begin();
         itConstraint != setUnionedConstraints.end(); itConstraint++)
    {
        int constraintID = *itConstraint ;
        auto pCIt = mapConstraintID2Info.find(constraintID) ;
        if ( pCIt == mapConstraintID2Info.end() ){
            puts("can not find constraint in updateVertexAndConstraints") ;
            continue ;
        }
        if ( pCIt->second.kind == CON_KF && pCIt->second.pose->isBad() ){//the keyframe is deleted or not
            continue ;
        }
        int vertexID = pCIt->second.vertexID ;

        auto pPIt = mapPointID2Info.find(vertexID) ;
        if ( pPIt == mapPointID2Info.end() ){
            puts("can not find point in updateVertexAndConstraints, 222") ;
            continue ;
        }
        Delaunay3::Vertex_handle hndlQ = pPIt->second.handle ;
        Segment QO = Segment(hndlQ->point(),
                             PointD3(pCIt->second.pose3d_old_x,
                                     pCIt->second.pose3d_old_y,
                                     pCIt->second.pose3d_old_z));
        markTetrahedraCrossingConstraintWithBookKeeping(hndlQ, QO, constraintID) ;
    }
}

void FreespaceDelaunayAlgorithm::addVertex(int vertexID, float x, float y, float z, set<int>& setUnionedConstraints )
{
    pointInfo currentPoint ;

    currentPoint.p3d_old_x = x ;
    currentPoint.p3d_new_x = x ;
    currentPoint.p3d_old_y = y ;
    currentPoint.p3d_new_y = y ;
    currentPoint.p3d_old_z = z ;
    currentPoint.p3d_new_z = z ;
    currentPoint.updateFlag = false ;
    currentPoint.vertexID = vertexID ;
    currentPoint.constraintIDList.clear();

    // Locate the point
    PointD3 Q(x, y, z) ;
    Delaunay3::Locate_type lt;
    int li, lj;
    Delaunay3::Cell_handle c = dt.locate(Q, lt, li, lj);
    if (lt == Delaunay3::VERTEX) {
        //cerr << "Error in FreespaceDelaunayAlgorithm::addVertex(): Attempted to add a duplicate vertex to the triangulation" << endl;
        return;
    }

    // Get the cells that conflict with Q in a vector vecConflictCells, and a facet on the boundary of this hole in f.
    vector<Delaunay3::Cell_handle> vecConflictCells;
    Delaunay3::Facet f;
    dt.find_conflicts(Q, c, CGAL::Oneset_iterator<Delaunay3::Facet>(f), std::back_inserter(vecConflictCells));

    // Get the unioned constraint set of all the cells in vecConflictCells.
    for (vector<Delaunay3::Cell_handle>::const_iterator it = vecConflictCells.begin();
         it != vecConflictCells.end(); it++)
    {
        list<int>& constrintIDList = (*it)->info().constrintIDList ;
        for( auto constrintID: constrintIDList )
        {
            setUnionedConstraints.insert(constrintID);
            auto pCIt = mapConstraintID2Info.find(constrintID) ;
            if ( pCIt == mapConstraintID2Info.end() )
            {
                puts("can not find constraint") ;
                return ;
            }
            pCIt->second.listCell.remove( &((*it)->info()) ) ;
        }
    }

    // Delete the cells in conflict, insert the point, and star off the hole.
    Delaunay3::Vertex_handle hndlQ =
            dt.insert_in_hole(Q, vecConflictCells.begin(), vecConflictCells.end(), f.first, f.second);
    currentPoint.handle = hndlQ ;

    std::pair<int,pointInfo> keyVertex(vertexID, currentPoint);
    mapPointID2Info.insert(keyVertex) ;
}

void FreespaceDelaunayAlgorithm::updateVertexAndConstraints(list<collectedPoint>& updatedPointsAndConstraints,
                                                            KeyFrame* kF)
{
    cv::Mat currentPose = kF->GetPoseInverse();
    float center_x = currentPose.at<float>(0,3) ;
    float center_y = currentPose.at<float>(1,3) ;
    float center_z = currentPose.at<float>(2,3) ;
    PointD3 O(center_x, center_y, center_z);
    set<int> setUnionedConstraints ;

    //insert vertexs
    for(list<collectedPoint>::iterator iter=updatedPointsAndConstraints.begin();
        iter != updatedPointsAndConstraints.end(); iter++ )
    {
        if ( iter->flag ){
            addVertex(iter->id , iter->x, iter->y, iter->z, setUnionedConstraints ) ;
        }
    }

    //insert the deleted constraints
    addSetOfConstraints(setUnionedConstraints) ;
    //    for (set<int>::iterator itConstraint = setUnionedConstraints.begin();
    //         itConstraint != setUnionedConstraints.end(); itConstraint++)
    //    {
    //        int constraintID = *itConstraint ;
    //        auto pCIt = mapConstraintID2Info.find(constraintID) ;
    //        if ( pCIt == mapConstraintID2Info.end() ){
    //            puts("can not find constraint in updateVertexAndConstraints") ;
    //            continue ;
    //        }
    //        if ( pCIt->second.kind == CON_KF && pCIt->second.pose->isBad() ){//the keyframe is deleted or not
    //            continue ;
    //        }
    //        int vertexID = pCIt->second.vertexID ;

    //        auto pPIt = mapPointID2Info.find(vertexID) ;
    //        if ( pPIt == mapPointID2Info.end() ){
    //            puts("can not find point in updateVertexAndConstraints, 222") ;
    //            continue ;
    //        }
    //        Delaunay3::Vertex_handle hndlQ = pPIt->second.handle ;
    //        Segment QO = Segment(hndlQ->point(),
    //                             PointD3(pCIt->second.pose3d_old_x,
    //                                     pCIt->second.pose3d_old_y,
    //                                     pCIt->second.pose3d_old_z));
    //        markTetrahedraCrossingConstraintWithBookKeeping(hndlQ, QO, constraintID) ;
    //    }

    //insert the currrent view constraints
    for(list<collectedPoint>::iterator iter=updatedPointsAndConstraints.begin();
        iter != updatedPointsAndConstraints.end(); iter++ )
    {
        auto pPIt = mapPointID2Info.find(iter->id) ;
        if ( pPIt == mapPointID2Info.end() ){
            //puts("can not find point in updateVertexAndConstraints, 222") ;
            continue ;
        }
        addConstraintKF(iter->id, kF);
    }
}


void FreespaceDelaunayAlgorithm::removeVertexSet( list<int>& currentOutlierVertexList, std::set<int>& setUnionedConstraints )
{
    // Vertex Deletion Algorithm:
    // ~~~~~~~~~~~~~~~~~~~~
    // Step 1: Collect FS constraints into a unioned set from incident cells.  Don't add FS constraints containing the vertex to be deleted.
    // Step 2: Delete the vertex (this retriangulates).
    // Step 3: Iterate over all cells and remove any FS constraints containing the deleted vertex.  Meanwhile determine the set of new cells.
    // Step 4: Process the FS Constraints in the unioned set.  Mark new cells as old.
    for( auto vertexID: currentOutlierVertexList )
    {
        auto pPIt = mapPointID2Info.find(vertexID) ;
        if ( pPIt == mapPointID2Info.end() )
        {
            puts("can not find point in removeVertex") ;
            return ;
        }
        // Step 0:
        std::list<int> constraintIDList(pPIt->second.constraintIDList);
        for( auto constrainID: constraintIDList ){
            //printf("constrainID = %d\n", constrainID) ;
            removeConstraint(constrainID);
        }

        Delaunay3::Vertex_handle hndlQ = pPIt->second.handle;

        vector<Delaunay3::Cell_handle> setIncidentCells;
        dt.incident_cells(hndlQ, std::inserter(setIncidentCells, setIncidentCells.begin()));

        // Step 1:
        for (vector<Delaunay3::Cell_handle>::const_iterator it = setIncidentCells.begin();
             it != setIncidentCells.end(); it++)
        {
            list<int>& constrintIDList = (*it)->info().constrintIDList ;
            for( auto id: constrintIDList )
            {
                //printf("id=%d\n", id ) ;
                auto pCIt = mapConstraintID2Info.find(id) ;
                if ( pCIt == mapConstraintID2Info.end() )
                {
                    puts("can not find constraint in removeVertex") ;
                    continue ;
                }
                pCIt->second.listCell.remove( &(*it)->info() ) ;
                if ( pCIt->second.vertexID == vertexID ){
                    continue ;
                }
                if ( pCIt->second.kind == CON_KF && pCIt->second.pose->isBad() ){//the keyframe is deleted or not
                    continue ;
                }
                setUnionedConstraints.insert(id);
            }
        }

        // Step 2:
        dt.remove(hndlQ);
        mapPointID2Info.erase(pPIt) ;
    }

    // Step 3:
    addSetOfConstraints(setUnionedConstraints);
    //    for( set<int>::iterator iter = setUnionedConstraints.begin();
    //         iter != setUnionedConstraints.end(); iter++ )
    //    {
    //        int constraintID = *iter ;
    //        auto pCIt = mapConstraintID2Info.find(constraintID) ;
    //        if ( pCIt == mapConstraintID2Info.end() )
    //        {
    //            //puts("can not find constraint") ;
    //            return ;
    //        }
    //        int vertexID = pCIt->second.vertexID ;
    //        auto pPIt = mapPointID2Info.find(vertexID) ;
    //        if ( pPIt == mapPointID2Info.end() )
    //        {
    //            puts("can not find point") ;
    //            return ;
    //        }
    //        Delaunay3::Vertex_handle handle = pPIt->second.handle;
    //        Segment QO = Segment(handle->point(),
    //                             PointD3(pCIt->second.pose3d_old_x,
    //                                     pCIt->second.pose3d_old_y,
    //                                     pCIt->second.pose3d_old_z));
    //        markTetrahedraCrossingConstraintWithBookKeeping(handle, QO, constraintID ) ;
    //    }
}

void FreespaceDelaunayAlgorithm::removeVertex_origin( int vertexID )
{
    // Vertex Deletion Algorithm:
    // ~~~~~~~~~~~~~~~~~~~~
    // Step 1: Collect FS constraints into a unioned set from incident cells.  Don't add FS constraints containing the vertex to be deleted.
    // Step 2: Delete the vertex (this retriangulates).
    // Step 3: Iterate over all cells and remove any FS constraints containing the deleted vertex.  Meanwhile determine the set of new cells.
    // Step 4: Process the FS Constraints in the unioned set.  Mark new cells as old.

    auto pPIt = mapPointID2Info.find(vertexID) ;
    if ( pPIt == mapPointID2Info.end() )
    {
        puts("can not find point in removeVertex") ;
        return ;
    }
    // Step 0:
    std::list<int> constraintIDList(pPIt->second.constraintIDList);
    for( auto constrainID: constraintIDList ){
        //printf("constrainID = %d\n", constrainID) ;
        removeConstraint(constrainID);
    }

    Delaunay3::Vertex_handle hndlQ = pPIt->second.handle;

    vector<Delaunay3::Cell_handle> setIncidentCells;
    dt.incident_cells(hndlQ, std::inserter(setIncidentCells, setIncidentCells.begin()));

    // Step 1:
    std::set<int> setUnionedConstraints ;
    for (vector<Delaunay3::Cell_handle>::const_iterator it = setIncidentCells.begin();
         it != setIncidentCells.end(); it++)
    {
        list<int>& constrintIDList = (*it)->info().constrintIDList ;
        for( auto id: constrintIDList )
        {
            //printf("id=%d\n", id ) ;
            auto pCIt = mapConstraintID2Info.find(id) ;
            if ( pCIt == mapConstraintID2Info.end() )
            {
                puts("can not find constraint in removeVertex") ;
                continue ;
            }
            pCIt->second.listCell.remove( &(*it)->info() ) ;
            if ( pCIt->second.vertexID == vertexID ){
                continue ;
            }
            if ( pCIt->second.kind == CON_KF && pCIt->second.pose->isBad() ){//the keyframe is deleted or not
                continue ;
            }
            setUnionedConstraints.insert(id);
        }
    }

    // Step 2:
    dt.remove(hndlQ);

    //    set<Delaunay3::Cell_handle> setNewCells;
    //    for (Delaunay3::Finite_cells_iterator itCell = dt.finite_cells_begin();
    //         itCell != dt.finite_cells_end(); itCell++ )
    //    {
    //        if (itCell->info().isNew()){
    //            setNewCells.insert(itCell);
    //        }
    //        // Linear search:
    //        std::list<int>& constraintIDList(pPIt->second.constraintIDList);
    //        for( auto constrainID: constraintIDList )
    //        {
    //            bool flag = false ;
    //            for ( auto id : itCell->info().constrintIDList )
    //            {
    //                if ( id == constrainID ){
    //                    flag = true ;
    //                    break ;
    //                }
    //            }
    //            if ( flag ){
    //                //printf("id=%d ", itCell->info().id) ;
    //                itCell->info().constrintIDList.remove(constrainID) ;
    //            }
    //        }
    //    }

    mapPointID2Info.erase(pPIt) ;
    //puts("step 2!") ;

    // Step 3:
    for( set<int>::iterator iter = setUnionedConstraints.begin();
         iter != setUnionedConstraints.end(); iter++ )
    {
        int constraintID = *iter ;
        auto pCIt = mapConstraintID2Info.find(constraintID) ;
        if ( pCIt == mapConstraintID2Info.end() )
        {
            puts("can not find constraint") ;
            return ;
        }
        int vertexID = pCIt->second.vertexID ;
        auto pPIt = mapPointID2Info.find(vertexID) ;
        if ( pPIt == mapPointID2Info.end() )
        {
            puts("can not find point") ;
            return ;
        }
        Delaunay3::Vertex_handle handle = pPIt->second.handle;
        Segment QO = Segment(handle->point(),
                             PointD3(pCIt->second.pose3d_old_x,
                                     pCIt->second.pose3d_old_y,
                                     pCIt->second.pose3d_old_z));
        markTetrahedraCrossingConstraintWithBookKeeping(handle, QO, constraintID ) ;
    }

    //puts("step 3!") ;
}


void FreespaceDelaunayAlgorithm::moveConstraint( int constraintID )
{

}


void FreespaceDelaunayAlgorithm::removeConstraint( int constraintID )
{
    auto pCIt = mapConstraintID2Info.find(constraintID) ;
    if ( pCIt == mapConstraintID2Info.end() )
    {
        puts("can not find constraint in removeConstraint") ;
        return ;
    }

    //remove related to cells
    for( list<Delaunay3CellInfo*>::iterator iter = pCIt->second.listCell.begin() ;
         iter != pCIt->second.listCell.end(); iter++ )
    {
        //printf("id=%d ", (*iter)->id ) ;
        (*iter)->constrintIDList.remove(constraintID) ;
    }
    //    printf("\n--------\n") ;
    //    //remove related to cells
    //    for (Delaunay3::Finite_cells_iterator itCell = dt.finite_cells_begin();
    //         itCell != dt.finite_cells_end(); itCell++ )
    //    {
    //        // Linear search:
    //        bool flag = false ;
    //        for ( auto id : itCell->info().constrintIDList )
    //        {
    //            if ( id == constraintID ){
    //                flag = true ;
    //                break ;
    //            }
    //        }
    //        if ( flag ){
    //            printf("id=%d ", itCell->info().id) ;
    //            itCell->info().constrintIDList.remove(constraintID) ;
    //        }
    //    }
    //    puts("") ;

    //puts("removeConstraint 0") ;

    //remove related to vertex
    int vertexID = pCIt->second.vertexID ;
    auto pPIt = mapPointID2Info.find(vertexID) ;
    if ( pPIt == mapPointID2Info.end() )
    {
        puts("can not find vertex in removeConstraint") ;
        return ;
    }
    pPIt->second.constraintIDList.remove(constraintID) ;

    //puts("removeConstraint 1") ;

    //remove related to pose
    KeyFrame* kF = pCIt->second.pose ;
    if ( pCIt->second.kind != CON_NONKF ){
        kF->constraintIDList.remove(constraintID);
    }
    //printf("[remove] cID=%d kFid=%d\n", constraintID, kF->mnId ) ;

    mapConstraintID2Info.erase(pCIt) ;
    //puts("removeConstraint 2") ;
}

void FreespaceDelaunayAlgorithm::tetsToTris(vector<dlovi::Matrix> & points, list<dlovi::Matrix> & tris)
{
    //int nVoteThresh = 1 ;
    // NEW Version, graph cut isosurf extraction with maxflow (builds the graph from scratch every time):
    {
        // Timing output for graphcuts:
        // cerr << "Running Graph Cut Isosurface Extraction..." << endl;
        //double t = timestamp();

        //tetsToTris_maxFlowSimple(points, tris, nVoteThresh);

        // cerr << "Time Taken (Isosurface): " << (timestamp() - t) << " s" << endl;
    }

    // OLD Version, simple isosurf extraction:
    //tetsToTris_naive(points, tris, nVoteThresh);
}

void FreespaceDelaunayAlgorithm::markTetrahedraCrossingConstraintWithBookKeeping(
        const Delaunay3::Vertex_handle& hndlQ, const Segment & constraint, int constraintID )
{
    auto pCIt = mapConstraintID2Info.find(constraintID) ;
    if ( pCIt == mapConstraintID2Info.end() ){
        puts("can not find constraint in markTetrahedraCrossingConstraint") ;
        return ;
    }

    Delaunay3::Cell_handle tetPrev;
    Delaunay3::Cell_handle tetCur;
    Delaunay3::Locate_type lt;
    int li, lj;

    dlovi::Matrix matQ(3, 1);
    dlovi::Matrix matO(3, 1);
    matQ(0) = constraint.source().x();
    matQ(1) = constraint.source().y();
    matQ(2) = constraint.source().z();
    matO(0) = constraint.target().x();
    matO(1) = constraint.target().y();
    matO(2) = constraint.target().z();

    // For all tetrahedra t incident to Q:
    vector<Delaunay3::Cell_handle> vecQCells;
    dt.incident_cells(hndlQ, std::back_inserter(vecQCells));
    vector<Delaunay3::Cell_handle>::iterator itQCells;
    for (itQCells = vecQCells.begin(); itQCells != vecQCells.end(); itQCells++)
    {
        // If t contains O:
        if (dt.side_of_cell(constraint.target(), *itQCells, lt, li, lj) != CGAL::ON_UNBOUNDED_SIDE)
        {
            bool flag = (*itQCells)->info().addIntersection(constraintID); // t.n++
            if ( flag ){
                pCIt->second.listCell.push_back( &( (*itQCells)->info()) ) ;
            }
            // We're done, so return
            return;
        }
        // Let f be the facet of t opposite to Q
        int f = (*itQCells)->index(hndlQ); // this with the Cell_handle *itQCells defines the facet
        // If f intersects QO
        if (CGAL::do_intersect(dt.triangle(*itQCells, f), constraint))
        {
            //if (triangleConstraintIntersectionTest(Delaunay3::Facet(*itQCells, f), matQ, matO)) {
            tetPrev = *itQCells; // t.precedent = t
            tetCur = (*itQCells)->neighbor(f); // t.actual = neighbour of t incident to facet f

            bool flag ;
            flag = (*itQCells)->info().addIntersection(constraintID);  // t.n++
            if ( flag ){
                pCIt->second.listCell.push_back( &((*itQCells)->info()) ) ;
            }
            flag = tetCur->info().addIntersection(constraintID); // t.actual.n++
            if ( flag ){
                pCIt->second.listCell.push_back( &(tetCur->info()) ) ;
            }
            break;
        }
    }
    // While t.actual doesn't contain O
    int f;

    //while(dt.side_of_cell(constraint.target(), tetCur, lt, li, lj) == CGAL::ON_UNBOUNDED_SIDE){
    while (cellTraversalExitTest(f, tetCur, tetPrev, matQ, matO))
    {
        // f is now the facet of t.actual that intersects QO and that isn't incident to t.precedent

        //for (f = 0; f < 4; f++) {
        //  if (tetCur->neighbor(f) == tetPrev) continue;
        //  if (CGAL::do_intersect(dt.triangle(tetCur, f), constraint)) break;
        //}

        tetPrev = tetCur; // t.precedent = t.actual
        tetCur = tetCur->neighbor(f); // t.actual = neighbour of t.precedent(==t.actual) incident to facet f
        bool flag = tetCur->info().addIntersection(constraintID);  // t.actual.n++
        if ( flag ){
            pCIt->second.listCell.push_back( &(tetCur->info()) ) ;
        }
    }

    //	 We find intersecting tetrahedra by Pau's algorithm:
    //	 let Q be the point & O the optic center.
    //	 For all tetrahedra t incident to Q:
    //		if t contains O:
    //			t.n++
    //			break maybe?  bad translation.  In this case should be ~done~ here anyways.
    //		let f be the facet	 of t opposite to Q
    //		if f intersects QO:
    //			t.precedent = t
    //			t.actual = neighbour of t incident to facet f
    //			t.n++;
    //			t.actual.n++;
    //			break from for loop
    //	 while t.actual doesn't contain O
    //		let f be the facet of t.actual that intersects QO and which isn't incident to t.precedent
    //		t.precedent = t.actual
    //		t.actual = neighbour of t.precedent(==t.actual) incident to facet f
    //		t.actual.n++;

    // Basically Pau's algorithm says:
    // Traverse the tetrahedrization.  For all traversed tetrahedra, increment the count.
    // Start with the tetrahedron that contains Q.  Do this by the algorithm of localization of a point in a delaunay triangulization.
    // (Here, this is modified by use of a triangulation hierarchy from CGAL for fast point localization.)
    // Loop over all neighbouring tetrahedra and find the one whose common facet intersects the segment QO.  Traverse to this.
    // Continue traversing adjacent tetrahedra by the constraint-intersecting facets.
    // Stop traversal when O is contained in the current tetrahedron.
}

bool FreespaceDelaunayAlgorithm::triangleConstraintIntersectionTest(const Delaunay3::Facet & tri,
                                                                    const dlovi::Matrix & segSrc,
                                                                    const dlovi::Matrix & segDest)
{
    // A custom implementation of the ray-triangle intersection test at http://jgt.akpeters.com/papers/MollerTrumbore97/code.html
    // Follows the back-face culling branch (ie: a triangle won't intersect the ray if the ray pierces the backside of it.)
    dlovi::Matrix edge1, edge2, tvec, pvec, qvec;
    double det, inv_det;
    double t, u, v;

    // Get the 3 triangle vertices
    dlovi::Matrix v0(3, 1);
    dlovi::Matrix v1(3, 1);
    dlovi::Matrix v2(3, 1);

    // Note:
    // tri.first = the Cell_handle containing the triangle
    // tri.second = f, the face index for tri.first
    // We want all normals pointing inward, ie positive halfspace of a triangle contains the 4th point of the tetrahedron.  So:
    // f == 0 -> (1, 3, 2)
    // f == 1 -> (0, 2, 3)
    // f == 2 -> (3, 1, 0)
    // f == 3 -> (0, 1, 2)
    if (tri.second == 3) {
        v0(0) = tri.first->vertex(0)->point().x(); v0(1) = tri.first->vertex(0)->point().y(); v0(2) = tri.first->vertex(0)->point().z();
        v1(0) = tri.first->vertex(1)->point().x(); v1(1) = tri.first->vertex(1)->point().y(); v1(2) = tri.first->vertex(1)->point().z();
        v2(0) = tri.first->vertex(2)->point().x(); v2(1) = tri.first->vertex(2)->point().y(); v2(2) = tri.first->vertex(2)->point().z();
    }
    else if (tri.second == 2) {
        v0(0) = tri.first->vertex(3)->point().x(); v0(1) = tri.first->vertex(3)->point().y(); v0(2) = tri.first->vertex(3)->point().z();
        v1(0) = tri.first->vertex(1)->point().x(); v1(1) = tri.first->vertex(1)->point().y(); v1(2) = tri.first->vertex(1)->point().z();
        v2(0) = tri.first->vertex(0)->point().x(); v2(1) = tri.first->vertex(0)->point().y(); v2(2) = tri.first->vertex(0)->point().z();
    }
    else if (tri.second == 1) {
        v0(0) = tri.first->vertex(0)->point().x(); v0(1) = tri.first->vertex(0)->point().y(); v0(2) = tri.first->vertex(0)->point().z();
        v1(0) = tri.first->vertex(2)->point().x(); v1(1) = tri.first->vertex(2)->point().y(); v1(2) = tri.first->vertex(2)->point().z();
        v2(0) = tri.first->vertex(3)->point().x(); v2(1) = tri.first->vertex(3)->point().y(); v2(2) = tri.first->vertex(3)->point().z();
    }
    else if (tri.second == 0) { // f == 0
        v0(0) = tri.first->vertex(1)->point().x(); v0(1) = tri.first->vertex(1)->point().y(); v0(2) = tri.first->vertex(1)->point().z();
        v1(0) = tri.first->vertex(3)->point().x(); v1(1) = tri.first->vertex(3)->point().y(); v1(2) = tri.first->vertex(3)->point().z();
        v2(0) = tri.first->vertex(2)->point().x(); v2(1) = tri.first->vertex(2)->point().y(); v2(2) = tri.first->vertex(2)->point().z();
    }
    else {
        cerr << "whaomg" << endl;
    }

    // Get the constraint ray's normalized direction vector
    dlovi::Matrix dir = segDest - segSrc;
    double dirNorm = dir.norm();
    dir /= dirNorm;

    // Find vectors for two edges sharing v0:
    edge1 = v1 - v0;
    edge2 = v2 - v0;

    // Begin calculating determinant - also used to calculate U parameter
    pvec = dir.cross(edge2);

    // If determinant is near zero, ray lies in plane of triangle.  We do backface culling for the intersection test,
    // so only need to check 1 halfspace
    det = edge1.dot(pvec);
    if (det < dlovi::sqrt_eps_d)
        return false;

    // Calculate distance from v0 to ray origin
    tvec = segSrc - v0;

    // Calculate U parameter and test bounds
    u = tvec.dot(pvec);
    if (u < 0.0 || u > det)
        return false;

    // Prepare to test V parameter
    qvec = tvec.cross(edge1);

    // Calculate V parameter and test bounds
    v = dir.dot(qvec);
    if (v < 0.0 || (u + v) > det)
        return false;

    // Calculate t, scale parameters, ray intersects triangle
    t = edge2.dot(qvec);
    inv_det = 1.0 / det;
    t *= inv_det;
    // u *= inv_det;
    // v *= inv_det;

    // Test if distance to plane t is too large and return false if so
    if (t < 0.0 || t > dirNorm)
        return false;

    return true;
}

bool FreespaceDelaunayAlgorithm::triangleConstraintIntersectionTest(bool & bCrossesInteriorOfConstraint,
                                                                    const vector<dlovi::Matrix> & points,
                                                                    const dlovi::Matrix & tri,
                                                                    const pair<dlovi::Matrix, dlovi::Matrix> & constraint)
{
    // A custom implementation of the ray-triangle intersection test at http://jgt.akpeters.com/papers/MollerTrumbore97/code.html
    // Follows the back-face culling branch (ie: a triangle won't intersect the ray if the ray pierces the backside of it.)
    dlovi::Matrix edge1, edge2, tvec, pvec, qvec;
    double det, inv_det;
    double t, u, v;

    // Set default for interiorOfConstraint = false return value (in the case that there is no intersection for quick returns)
    bCrossesInteriorOfConstraint = false;

    // Get the two segment points:
    const dlovi::Matrix & segSrc = constraint.first;
    const dlovi::Matrix & segDest = constraint.second;

    // Get the 3 triangle vertices
    const dlovi::Matrix & v0 = points[round(tri(0))];
    const dlovi::Matrix & v1 = points[round(tri(1))];
    const dlovi::Matrix & v2 = points[round(tri(2))];

    // Get the constraint ray's normalized direction vector
    dlovi::Matrix dir = segDest - segSrc;
    double dirNorm = dir.norm();
    dir /= dirNorm;

    // Find vectors for two edges sharing v0:
    edge1 = v1 - v0;
    edge2 = v2 - v0;

    // Begin calculating determinant - also used to calculate U parameter
    pvec = dir.cross(edge2);

    // If determinant is near zero, ray lies in plane of triangle.  We do backface culling for the intersection test,
    // so only need to check 1 halfspace
    det = edge1.dot(pvec);
    if (det < dlovi::sqrt_eps_d)
        return false;

    // Calculate distance from v0 to ray origin
    tvec = segSrc - v0;

    // Calculate U parameter and test bounds
    u = tvec.dot(pvec);
    if (u < 0.0 || u > det)
        return false;

    // Prepare to test V parameter
    qvec = tvec.cross(edge1);

    // Calculate V parameter and test bounds
    v = dir.dot(qvec);
    if (v < 0.0 || (u + v) > det)
        return false;

    // Calculate t, scale parameters, ray intersects triangle
    t = edge2.dot(qvec);
    inv_det = 1.0 / det;
    t *= inv_det;
    // u *= inv_det;
    // v *= inv_det;

    // Test if distance to plane t is too large and return false if so
    if (t < 0.0 || t > dirNorm)
        return false;
    if (t != 0.0 && t != dirNorm)
        bCrossesInteriorOfConstraint = true;

    return true;
}

bool FreespaceDelaunayAlgorithm::cellTraversalExitTest(int & f,
                                                       const Delaunay3::Cell_handle tetCur,
                                                       const Delaunay3::Cell_handle tetPrev,
                                                       const dlovi::Matrix & matQ,
                                                       const dlovi::Matrix & matO)
{
    // TODO: See if we can optimize this by reuse: we use the same constraint QO in all 3 face tests.  Faces also share edges and points.
    vector<dlovi::Matrix> points;
    dlovi::Matrix tri(3, 1);
    pair<dlovi::Matrix, dlovi::Matrix> constraint(matQ, matO);
    bool bCrossesInteriorOfConstraint;
    dlovi::Matrix tmpMat(3, 1);

    // Let f be the entry face's index.
    if (tetCur->neighbor(0) == tetPrev) f = 0;
    else if (tetCur->neighbor(1) == tetPrev) f = 1;
    else if (tetCur->neighbor(2) == tetPrev) f = 2;
    else if (tetCur->neighbor(3) == tetPrev) f = 3;

    // Collect the tetrahedra's 4 vertices into the variable points
    tmpMat(0) = tetCur->vertex(0)->point().x(); tmpMat(1) = tetCur->vertex(0)->point().y(); tmpMat(2) = tetCur->vertex(0)->point().z();
    points.push_back(tmpMat);
    tmpMat(0) = tetCur->vertex(1)->point().x(); tmpMat(1) = tetCur->vertex(1)->point().y(); tmpMat(2) = tetCur->vertex(1)->point().z();
    points.push_back(tmpMat);
    tmpMat(0) = tetCur->vertex(2)->point().x(); tmpMat(1) = tetCur->vertex(2)->point().y(); tmpMat(2) = tetCur->vertex(2)->point().z();
    points.push_back(tmpMat);
    tmpMat(0) = tetCur->vertex(3)->point().x(); tmpMat(1) = tetCur->vertex(3)->point().y(); tmpMat(2) = tetCur->vertex(3)->point().z();
    points.push_back(tmpMat);

    // Construct the indices for the triangles to test, and test them as needed.
    // We want all normals pointing inward, ie positive halfspace of a triangle contains the 4th point of the tetrahedron.  So:
    // f == 0 -> (1, 3, 2)
    // f == 1 -> (0, 2, 3)
    // f == 2 -> (3, 1, 0)
    // f == 3 -> (0, 1, 2)
    if (f == 3) {
        // Test face 0
        tri(0) = 1; tri(1) = 3; tri(2) = 2;
        if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
            f = 0;
            return bCrossesInteriorOfConstraint;
        }
        // Test face 1
        tri(0) = 0; tri(1) = 2; tri(2) = 3;
        if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
            f = 1;
            return bCrossesInteriorOfConstraint;
        }
        // Test face 2
        tri(0) = 3; tri(1) = 1; tri(2) = 0;
        if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
            f = 2;
            return bCrossesInteriorOfConstraint;
        }
    }
    else if (f == 2) {
        // Test face 0
        tri(0) = 1; tri(1) = 3; tri(2) = 2;
        if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
            f = 0;
            return bCrossesInteriorOfConstraint;
        }
        // Test face 1
        tri(0) = 0; tri(1) = 2; tri(2) = 3;
        if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
            f = 1;
            return bCrossesInteriorOfConstraint;
        }
        // Test face 3
        tri(0) = 0; tri(1) = 1; tri(2) = 2;
        if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
            f = 3;
            return bCrossesInteriorOfConstraint;
        }
    }
    else if (f == 1) {
        // Test face 0
        tri(0) = 1; tri(1) = 3; tri(2) = 2;
        if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
            f = 0;
            return bCrossesInteriorOfConstraint;
        }
        // Test face 2
        tri(0) = 3; tri(1) = 1; tri(2) = 0;
        if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
            f = 2;
            return bCrossesInteriorOfConstraint;
        }
        // Test face 3
        tri(0) = 0; tri(1) = 1; tri(2) = 2;
        if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
            f = 3;
            return bCrossesInteriorOfConstraint;
        }
    }
    else { // f == 0
        // Test face 1
        tri(0) = 0; tri(1) = 2; tri(2) = 3;
        if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
            f = 1;
            return bCrossesInteriorOfConstraint;
        }
        // Test face 2
        tri(0) = 3; tri(1) = 1; tri(2) = 0;
        if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
            f = 2;
            return bCrossesInteriorOfConstraint;
        }
        // Test face 3
        tri(0) = 0; tri(1) = 1; tri(2) = 2;
        if (triangleConstraintIntersectionTest(bCrossesInteriorOfConstraint, points, tri, constraint)) {
            f = 3;
            return bCrossesInteriorOfConstraint;
        }
    }
    // If no triangle intersects the constraint, then the optic center lies in the interior (or on the entry face) of the tetrahedron.  So return false.
    return false;
}

void FreespaceDelaunayAlgorithm::facetToTri(const Delaunay3::Facet & f,
                                            vector<Delaunay3::Vertex_handle> & vecTri)
{
    if (f.second == 0) {
        // Vertex handle order: 3 2 1
        vecTri.push_back(f.first->vertex(3));
        vecTri.push_back(f.first->vertex(2));
        vecTri.push_back(f.first->vertex(1));
    }
    else if (f.second == 1) {
        // Vertex handle order: 0 2 3
        vecTri.push_back(f.first->vertex(0));
        vecTri.push_back(f.first->vertex(2));
        vecTri.push_back(f.first->vertex(3));
    }
    else if (f.second == 2) {
        // Vertex handle order: 3 1 0
        vecTri.push_back(f.first->vertex(3));
        vecTri.push_back(f.first->vertex(1));
        vecTri.push_back(f.first->vertex(0));
    }
    else { // f->second == 3
        // Vertex handle order: 0 1 2
        vecTri.push_back(f.first->vertex(0));
        vecTri.push_back(f.first->vertex(1));
        vecTri.push_back(f.first->vertex(2));
    }
}

void FreespaceDelaunayAlgorithm::tetsToTris_naive(vector<myPoint3f> & points,
                                                  vector<myTriangles> & tris,
                                                  const int nVoteThresh)
{
    vector<Delaunay3::Vertex_handle> vecBoundsHandles;
    vector<Delaunay3::Vertex_handle> vecVertexHandles;
    hash_map<Delaunay3::Vertex_handle, int, HashVertHandle, EqVertHandle> hmapVertexHandleToIndex;

    // Initialize points and tris as empty:
    if (! points.empty()) points.clear();
    if (! tris.empty()) tris.clear();

    // Create a list of vertex handles to the bounding vertices (they'll be the vertices connected to the infinite vertex):
    dt.incident_vertices (dt.infinite_vertex(), std::back_inserter(vecBoundsHandles));

    // Populate the model's point list, create a list of finite non-bounding vertex handles, and
    // create a useful associative maps (handle->point list index).
    for (Delaunay3::Finite_vertices_iterator itVert = dt.finite_vertices_begin();
         itVert != dt.finite_vertices_end(); itVert++)
    {
        vector<Delaunay3::Vertex_handle>::iterator itBounds;
        for (itBounds = vecBoundsHandles.begin(); itBounds != vecBoundsHandles.end(); itBounds++) {
            if ((*itBounds) == ((Delaunay3::Vertex_handle)itVert))
                break;
        }
        if (itBounds == vecBoundsHandles.end()) { // the vertex is not a bounding vertex, so add it
            myPoint3f tmpPoint ;
            tmpPoint.x = itVert->point().x();
            tmpPoint.y = itVert->point().y();
            tmpPoint.z = itVert->point().z();
            points.push_back(tmpPoint);
            vecVertexHandles.push_back(itVert);
            hmapVertexHandleToIndex[itVert] = points.size() - 1;
        }
    }

    // Iterate over finite facets, and add a triangle to the mesh (w/ correct orientation).
    for (Delaunay3::Finite_facets_iterator itFacet = dt.finite_facets_begin();
         itFacet != dt.finite_facets_end(); itFacet++)
    {
        //        bool keepFlag = false ;
        //        for( int i=0; i < 4; i++ )
        //        {
        //            if ( i == itFacet->second ) continue;
        //            if (  itFacet->first->vertex(i)->point().y() > -2 ){
        //                keepFlag = true ;
        //                break ;
        //            }
        //        }
        //        if ( keepFlag == false ){
        //            continue ;
        //        }

        // 1. One adjacent cell is empty, and the other is not,
        // 2. The facet contains no vertex from the bounding vertices
        bool bFacetCellKept = itFacet->first->info().isKeptByVoteCount(nVoteThresh);
        bool bMirrorCellKept = itFacet->first->neighbor(itFacet->second)->info().isKeptByVoteCount(nVoteThresh);
        if (bFacetCellKept && ! bMirrorCellKept)
        {
            bool bContainsBoundsVert = false;
            for (int i = 0; i < 4; i++)
            {
                //if (i == itFacet->second) continue;
                if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
                    bContainsBoundsVert = true;
                    break;
                }
            }
            if (! bContainsBoundsVert)
            {
                Delaunay3::Facet fTmp = dt.mirror_facet(*itFacet); // The normal points inward so mirror the facet
                vector<Delaunay3::Vertex_handle> vecTri;

                facetToTri(fTmp, vecTri);
                myTriangles tmpTri ;
                tmpTri.p0 = points[hmapVertexHandleToIndex[vecTri[0]]];
                tmpTri.p1 = points[hmapVertexHandleToIndex[vecTri[1]]];
                tmpTri.p2 = points[hmapVertexHandleToIndex[vecTri[2]]];
                tris.push_back(tmpTri);
            }
        }
        else if (bMirrorCellKept && ! bFacetCellKept)
        {
            bool bContainsBoundsVert = false;
            for (int i = 0; i < 4; i++)
            {
                //if (i == itFacet->second) continue;
                if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0)
                {
                    bContainsBoundsVert = true;
                    break;
                }
            }
            if (! bContainsBoundsVert)
            {
                Delaunay3::Facet fTmp = *itFacet; // The normal points outward so no need to mirror the facet
                vector<Delaunay3::Vertex_handle> vecTri;

                facetToTri(fTmp, vecTri);
                myTriangles tmpTri ;
                tmpTri.p0 = points[hmapVertexHandleToIndex[vecTri[0]]];
                tmpTri.p1 = points[hmapVertexHandleToIndex[vecTri[1]]];
                tmpTri.p2 = points[hmapVertexHandleToIndex[vecTri[2]]];
                tris.push_back(tmpTri);
            }
        }
    }
}

void FreespaceDelaunayAlgorithm::edgesSet( vector<pair<float,float>>& edges )
{
    vector<Delaunay3::Vertex_handle> vecBoundsHandles;
    // Create a list of vertex handles to the bounding vertices (they'll be the vertices connected to the infinite vertex):
    dt.incident_vertices (dt.infinite_vertex(), std::back_inserter(vecBoundsHandles));

    //    edges.clear();
    //    edges.reserve( dt.number_of_finite_edges() + 5 );
    //    // Iterate over finite facets, and add a triangle to the mesh (w/ correct orientation).
    //    for (Delaunay3::Finite_facets_iterator itFacet = dt.finite_facets_begin();
    //         itFacet != dt.finite_facets_end(); itFacet++)
    //    {
    //        vector<Delaunay3::Vertex_handle>::iterator itBounds;
    //        for (itBounds = vecBoundsHandles.begin(); itBounds != vecBoundsHandles.end(); itBounds++) {
    //            if ((*itBounds) == ((Delaunay3::Vertex_handle)itVert))
    //                break;
    //        }


    //        // 1. One adjacent cell is empty, and the other is not,
    //        // 2. The facet contains no vertex from the bounding vertices
    //        bool bFacetCellKept = itFacet->first->info().empty;
    //        bool bMirrorCellKept = itFacet->first->neighbor(itFacet->second)->info().empty;
    //        if (bFacetCellKept && ! bMirrorCellKept)
    //        {
    //            bool bContainsBoundsVert = false;
    //            for (int i = 0; i < 4; i++)
    //            {
    //                //if (i == itFacet->second) continue;
    //                if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
    //                    bContainsBoundsVert = true;
    //                    break;
    //                }
    //            }
    //            if (! bContainsBoundsVert)
    //            {
    //                Delaunay3::Facet fTmp = dt.mirror_facet(*itFacet); // The normal points inward so mirror the facet
    //                vector<Delaunay3::Vertex_handle> vecTri;

    //                facetToTri(fTmp, vecTri);
    //                myTriangles tmpTri ;
    //                tmpTri.p0 = points[hmapVertexHandleToIndex[vecTri[0]]];
    //                tmpTri.p1 = points[hmapVertexHandleToIndex[vecTri[1]]];
    //                tmpTri.p2 = points[hmapVertexHandleToIndex[vecTri[2]]];
    //                tris.push_back(tmpTri);
    //            }
    //        }
    //        else if (bMirrorCellKept && ! bFacetCellKept)
    //        {
    //            bool bContainsBoundsVert = false;
    //            for (int i = 0; i < 4; i++)
    //            {
    //                //if (i == itFacet->second) continue;
    //                if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0)
    //                {
    //                    bContainsBoundsVert = true;
    //                    break;
    //                }
    //            }
    //            if (! bContainsBoundsVert)
    //            {
    //                Delaunay3::Facet fTmp = *itFacet; // The normal points outward so no need to mirror the facet
    //                vector<Delaunay3::Vertex_handle> vecTri;

    //                facetToTri(fTmp, vecTri);
    //                myTriangles tmpTri ;
    //                tmpTri.p0 = points[hmapVertexHandleToIndex[vecTri[0]]];
    //                tmpTri.p1 = points[hmapVertexHandleToIndex[vecTri[1]]];
    //                tmpTri.p2 = points[hmapVertexHandleToIndex[vecTri[2]]];
    //                tris.push_back(tmpTri);
    //            }
    //        }
    //    }
}


void FreespaceDelaunayAlgorithm::tetsToTris_naive2(vector<myPoint3f> & points,
                                                   vector<myTriangles> & tris,
                                                   vector<pair<myPoint3f,myPoint3f>> & edges,
                                                   const int nVoteThresh)
{
    vector<Delaunay3::Vertex_handle> vecBoundsHandles;
    vector<Delaunay3::Vertex_handle> vecVertexHandles;
    hash_map<Delaunay3::Vertex_handle, int, HashVertHandle, EqVertHandle> hmapVertexHandleToIndex;

    // Initialize points and tris as empty:
    if (! points.empty()) points.clear();
    if (! tris.empty()) tris.clear();
    if (! edges.empty()) edges.clear();
    edges.reserve( dt.number_of_finite_edges() + 5 );

    // Create a list of vertex handles to the bounding vertices (they'll be the vertices connected to the infinite vertex):
    dt.incident_vertices (dt.infinite_vertex(), std::back_inserter(vecBoundsHandles));

    // Populate the model's point list, create a list of finite non-bounding vertex handles, and
    // create a useful associative maps (handle->point list index).
    for (Delaunay3::Finite_vertices_iterator itVert = dt.finite_vertices_begin();
         itVert != dt.finite_vertices_end(); itVert++)
    {
        vector<Delaunay3::Vertex_handle>::iterator itBounds;
        for (itBounds = vecBoundsHandles.begin(); itBounds != vecBoundsHandles.end(); itBounds++) {
            if ((*itBounds) == ((Delaunay3::Vertex_handle)itVert))
                break;
        }
        if (itBounds == vecBoundsHandles.end()) { // the vertex is not a bounding vertex, so add it
            myPoint3f tmpPoint ;
            tmpPoint.x = itVert->point().x();
            tmpPoint.y = itVert->point().y();
            tmpPoint.z = itVert->point().z();
            points.push_back(tmpPoint);
            vecVertexHandles.push_back(itVert);
            hmapVertexHandleToIndex[itVert] = points.size() - 1;
        }
    }

    for (Delaunay3::Finite_cells_iterator itCell = dt.finite_cells_begin();
         itCell != dt.finite_cells_end(); itCell++)
    {
        itCell->info().empty = itCell->info().isKeptByVoteCount(nVoteThresh);
    }

    //    for (Delaunay3::Finite_cells_iterator itCell = dt.finite_cells_begin();
    //         itCell != dt.finite_cells_end(); itCell++)
    //    {
    //        if ( itCell->info().empty ) {
    //            continue ;
    //        }
    //        int sum = 0 ;
    //        for( int i = 0 ;i < 4 ; i++ ){
    //            sum += itCell->neighbor(i)->info().empty;
    //        }
    //        if ( sum >= 3 ){
    //            itCell->info().empty = true ;
    //        }
    //    }
    //    for (Delaunay3::Finite_cells_iterator itCell = dt.finite_cells_begin();
    //         itCell != dt.finite_cells_end(); itCell++)
    //    {
    //        if ( itCell->info().empty ) {
    //            continue ;
    //        }
    //        int sum = 0 ;
    //        for( int i = 0 ;i < 4 ; i++ ){
    //            sum += itCell->neighbor(i)->info().empty;
    //        }
    //        if ( sum >= 3 ){
    //            itCell->info().empty = true ;
    //        }
    //    }


    // Iterate over finite facets, and add a triangle to the mesh (w/ correct orientation).
    for (Delaunay3::Finite_facets_iterator itFacet = dt.finite_facets_begin();
         itFacet != dt.finite_facets_end(); itFacet++)
    {
        //        bool keepFlag = false ;
        //        for( int i=0; i < 4; i++ )
        //        {
        //            if ( i == itFacet->second ) continue;
        //            if (  itFacet->first->vertex(i)->point().y() > -2 ){
        //                keepFlag = true ;
        //                break ;
        //            }
        //        }
        //        if ( keepFlag == false ){
        //            continue ;
        //        }

        // 1. One adjacent cell is empty, and the other is not,
        // 2. The facet contains no vertex from the bounding vertices
        bool bFacetCellKept = itFacet->first->info().empty;
        bool bMirrorCellKept = itFacet->first->neighbor(itFacet->second)->info().empty;
        if (bFacetCellKept && ! bMirrorCellKept)
        {
            bool bContainsBoundsVert = false;
            for (int i = 0; i < 4; i++)
            {
                //if (i == itFacet->second) continue;
                if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
                    bContainsBoundsVert = true;
                    break;
                }
            }
            if (! bContainsBoundsVert)
            {
                Delaunay3::Facet fTmp = dt.mirror_facet(*itFacet); // The normal points inward so mirror the facet
                vector<Delaunay3::Vertex_handle> vecTri;

                facetToTri(fTmp, vecTri);
                myTriangles tmpTri ;
                tmpTri.p0 = points[hmapVertexHandleToIndex[vecTri[0]]];
                tmpTri.p1 = points[hmapVertexHandleToIndex[vecTri[1]]];
                tmpTri.p2 = points[hmapVertexHandleToIndex[vecTri[2]]];
                tris.push_back(tmpTri);
            }
        }
        else if (bMirrorCellKept && ! bFacetCellKept)
        {
            bool bContainsBoundsVert = false;
            for (int i = 0; i < 4; i++)
            {
                //if (i == itFacet->second) continue;
                if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0)
                {
                    bContainsBoundsVert = true;
                    break;
                }
            }
            if (! bContainsBoundsVert)
            {
                Delaunay3::Facet fTmp = *itFacet; // The normal points outward so no need to mirror the facet
                vector<Delaunay3::Vertex_handle> vecTri;

                facetToTri(fTmp, vecTri);
                myTriangles tmpTri ;
                tmpTri.p0 = points[hmapVertexHandleToIndex[vecTri[0]]];
                tmpTri.p1 = points[hmapVertexHandleToIndex[vecTri[1]]];
                tmpTri.p2 = points[hmapVertexHandleToIndex[vecTri[2]]];
                tris.push_back(tmpTri);
            }
        }

        bool bContainsBoundsVert = false;
        for (int i = 0; i < 4; i++)
        {
            //if (i == itFacet->second) continue;
            if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
                bContainsBoundsVert = true;
                break;
            }
        }
        if (! bContainsBoundsVert)
        {
            myPoint3f tmpPoint, tmpPoint2, tmpPoint3 ;

            Delaunay3::Facet fTmp = *itFacet;
            tmpPoint.x = fTmp.first->vertex(1)->point().x() ;
            tmpPoint.y = fTmp.first->vertex(1)->point().y() ;
            tmpPoint.z = fTmp.first->vertex(1)->point().z() ;

            tmpPoint2.x = fTmp.first->vertex(2)->point().x() ;
            tmpPoint2.y = fTmp.first->vertex(2)->point().y() ;
            tmpPoint2.z = fTmp.first->vertex(2)->point().z() ;

            tmpPoint3.x = fTmp.first->vertex(3)->point().x() ;
            tmpPoint3.y = fTmp.first->vertex(3)->point().y() ;
            tmpPoint3.z = fTmp.first->vertex(3)->point().z() ;

            std::pair<myPoint3f,myPoint3f> e0(tmpPoint, tmpPoint2);
            std::pair<myPoint3f,myPoint3f> e1(tmpPoint, tmpPoint3);
            std::pair<myPoint3f,myPoint3f> e2(tmpPoint2, tmpPoint3);

            edges.push_back(e0);
            edges.push_back(e1);
            //edges.push_back(e2);
        }
    }
}

void FreespaceDelaunayAlgorithm::tetsToFreeSpace(vector<myTriangles> & tris)
{
    int nVoteThresh = 1 ;
    vector<Delaunay3::Vertex_handle> vecBoundsHandles;
    // Initialize points and tris as empty:
    if (! tris.empty()) tris.clear();

    // Create a list of vertex handles to the bounding vertices (they'll be the vertices connected to the infinite vertex):
    dt.incident_vertices (dt.infinite_vertex(), std::back_inserter(vecBoundsHandles));

    myPoint3f p[4] ;
    for (Delaunay3::Finite_cells_iterator itCell = dt.finite_cells_begin();
         itCell != dt.finite_cells_end(); itCell++)
    {
        if ( itCell->info().isKeptByVoteCount(nVoteThresh) ){
            continue ;
        }
        for( int i = 0 ; i < 4; i++ )
        {
            bool bContainsBoundsVert = false;
            for (auto handle: vecBoundsHandles )
            {
                if (itCell->vertex(i) == handle ){
                    bContainsBoundsVert |= true;
                    break ;
                }
            }
            if ( bContainsBoundsVert ){
                break ;
            }
            p[i].x = itCell->vertex(i)->point().x();
            p[i].y = itCell->vertex(i)->point().y();
            p[i].z = itCell->vertex(i)->point().z();
        }
        for( int i=0; i < 2; i++ )
        {
            for( int j=i+1; j < 3; j++ )
            {
                for( int k=j+1; k < 4; k++ )
                {
                    myTriangles tmpTri ;
                    tmpTri.p0 = p[i];
                    tmpTri.p1 = p[j];
                    tmpTri.p2 = p[k];
                    tris.push_back(tmpTri);
                }
            }
        }
    }
}

void FreespaceDelaunayAlgorithm::tetsToTris_maxFlowSimple(vector<myPoint3f> & points,
                                                          vector<myTriangles> & tris,
                                                          const int nVoteThresh)
{
    vector<Delaunay3::Vertex_handle> vecBoundsHandles;
    vector<Delaunay3::Vertex_handle> vecVertexHandles;
    hash_map<Delaunay3::Vertex_handle, int, HashVertHandle, EqVertHandle> hmapVertexHandleToIndex;
    map<Delaunay3::Cell_handle, int> mapCellHandleToIndex;
    dlovi::Matrix matTmpPoint(3, 1);
    int loop;

    // Get some size-properties from the triangulation (non-constant-time access functions in the triangulation class)
    int numFiniteTets = dt.number_of_finite_cells();
    int numFiniteFacets = dt.number_of_finite_facets();

    // Initialize the maxflow graph
    Graph_t graph(numFiniteTets, numFiniteFacets);

    // Initialize points and tris as empty:
    if (! points.empty()) points.clear();
    if (! tris.empty()) tris.clear();

    // Create a list of vertex handles to the bounding vertices (they'll be the vertices connected to the infinite vertex):
    dt.incident_vertices (dt.infinite_vertex(), std::back_inserter(vecBoundsHandles));

    // Populate the model's point list, create a list of finite non-bounding vertex handles, and
    // create a useful associative map (handle->point list index).
    for (Delaunay3::Finite_vertices_iterator itVert = dt.finite_vertices_begin();
         itVert != dt.finite_vertices_end(); itVert++)
    {
        vector<Delaunay3::Vertex_handle>::iterator itBounds;
        for (itBounds = vecBoundsHandles.begin(); itBounds != vecBoundsHandles.end(); itBounds++) {
            if ((*itBounds) == ((Delaunay3::Vertex_handle)itVert))
                break;
        }
        if (itBounds == vecBoundsHandles.end()) { // the vertex is not a bounding vertex, so add it
            myPoint3f tmpPoint ;
            tmpPoint.x = itVert->point().x();
            tmpPoint.y = itVert->point().y();
            tmpPoint.z = itVert->point().z();
            points.push_back(tmpPoint);
            vecVertexHandles.push_back(itVert);
            hmapVertexHandleToIndex[itVert] = points.size() - 1;
        }
    }

    // Create useful associative maps (tet list index->handle & handle->tet list index).
    Delaunay3::Finite_cells_iterator it;
    for (loop = 0, it = dt.finite_cells_begin(); it != dt.finite_cells_end(); it++, loop++)
        mapCellHandleToIndex[it] = loop;

    // Add the source and sink to the graph
    graph.addSource();
    graph.addSink();

    // Construct the graph's edge costs to minimize an engergy E = data + lambda_smooth * smoothness:
    // Labels:
    //   source s (0) = outside
    //   sink t (1) = inside
    // Data term (TODO: tune these bogus params):
    //   P(constraint in x | x = outside) = 1
    //   P(no constraint in x | x = outside) = 0
    //   P(constraint in x | x = inside) = 0
    //   P(no constraint in x | x = inside) = 1
    const double P_constr_X0 = 1.0;
    const double P_no_constr_X0 = 0.0;
    const double P_constr_X1 = 0.0;
    const double P_no_constr_X1 = 1.0;
    const double lambda_smooth = 0.3; //0.75;  // Good values approx. < 0.5 to 1

    // Construct the graph's data terms
    for (it = dt.finite_cells_begin(); it != dt.finite_cells_end(); it++) {
        int node = mapCellHandleToIndex.find(it)->second;

        double tetVolume = fabs(dt.tetrahedron(it).volume());

        if (it->info().getVoteCount()) {
            // node X has constraints
            graph.addTWeights(node, P_constr_X0 * tetVolume, P_constr_X1 * tetVolume);
        }
        else {
            // node X has no constraint
            graph.addTWeights(node, P_no_constr_X0 * tetVolume, P_no_constr_X1 * tetVolume);
        }
    }

    // Iterate over finite facets to construct the graph's regularization
    for (Delaunay3::Finite_facets_iterator it = dt.finite_facets_begin(); it != dt.finite_facets_end(); it++) {
        // If the facet contains a bounding vert, it won't be added to the isosurface, so don't penalize it with a smoothness cost.
        bool bContainsBoundsVert = false;
        for (int i = 0; i < 4; i++) {
            if (i == it->second) continue;
            if (std::find(vecBoundsHandles.begin(), vecBoundsHandles.end(), it->first->vertex(i)) != vecBoundsHandles.end()) {
                bContainsBoundsVert = true;  break;
            }
        }
        if (! bContainsBoundsVert) {
            double smoothness_cost = lambda_smooth * sqrt(dt.triangle(*it).squared_area());
            graph.addEdge(mapCellHandleToIndex[it->first], mapCellHandleToIndex[it->first->neighbor(it->second)], smoothness_cost, smoothness_cost);
        }
    }

    // Run the maxflow algorithm to determine the labeling
    graph.maxflow();
    //double flow = graph.maxflow();
    //cerr << "Max Flow: " << flow << endl;

    // Iterate over finite facets to extract the mesh's triangles from the graph's mincut labeling
    for (Delaunay3::Finite_facets_iterator itFacet = dt.finite_facets_begin();
         itFacet != dt.finite_facets_end(); itFacet++)
    {
        // If one adjacent cell is empty, and the other is not, and if the facet contains no vertex from the bounding vertices,
        // then add a triangle to the mesh (w/ correct orientation).
        bool bFacetCellKept = ! graph.whatSegment(mapCellHandleToIndex[itFacet->first]);
        bool bMirrorCellKept = ! graph.whatSegment(mapCellHandleToIndex[itFacet->first->neighbor(itFacet->second)]);
        if (bFacetCellKept && ! bMirrorCellKept)
        {
            bool bContainsBoundsVert = false;
            for (int i = 0; i < 4; i++)
            {
                if (i == itFacet->second) continue;
                if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
                    bContainsBoundsVert = true;
                    break;
                }
            }
            if (! bContainsBoundsVert)
            {
                Delaunay3::Facet fTmp = dt.mirror_facet(*itFacet); // The normal points inward so mirror the facet
                vector<Delaunay3::Vertex_handle> vecTri;

                facetToTri(fTmp, vecTri);
                myTriangles tmpTri ;
                tmpTri.p0 = points[hmapVertexHandleToIndex[vecTri[0]]];
                tmpTri.p1 = points[hmapVertexHandleToIndex[vecTri[1]]];
                tmpTri.p2 = points[hmapVertexHandleToIndex[vecTri[2]]];
                tris.push_back(tmpTri);
            }
        }
        else if (bMirrorCellKept && ! bFacetCellKept)
        {
            bool bContainsBoundsVert = false;
            for (int i = 0; i < 4; i++)
            {
                if (i == itFacet->second) continue;
                if (hmapVertexHandleToIndex.count(itFacet->first->vertex(i)) == 0) {
                    bContainsBoundsVert = true;
                    break;
                }
            }
            if (! bContainsBoundsVert) {
                Delaunay3::Facet fTmp = *itFacet; // The normal points outward so no need to mirror the facet
                vector<Delaunay3::Vertex_handle> vecTri;

                facetToTri(fTmp, vecTri);
                myTriangles tmpTri ;
                tmpTri.p0 = points[hmapVertexHandleToIndex[vecTri[0]]];
                tmpTri.p1 = points[hmapVertexHandleToIndex[vecTri[1]]];
                tmpTri.p2 = points[hmapVertexHandleToIndex[vecTri[2]]];
                tris.push_back(tmpTri);
            }
        }
    }
}

} //namespace ORB_SLAM
