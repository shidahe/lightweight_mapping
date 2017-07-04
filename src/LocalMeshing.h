
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

#ifndef LOCALMESHING_H
#define LOCALMESHING_H

#include "KeyFrame.h"
#include "Map.h"
#include <vector>
#include <list>
#include <map>
#include <thread>
#include <mutex>
#include <cmath>
#include <unordered_map>
#include <tuple>
#include <unistd.h>

#include "dataStructure.h"
#include "FColorMap.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/StdVector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <string>
#include <set>
#include <map>
#include <utility>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <iostream>
#include <limits>
#include <ext/hash_map>
#include "delaunay/lovimath.h"
#include "delaunay/Matrix.h"
#include "delaunay/GraphWrapper_Boost.h"

// CGAL-related includes
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_hierarchy_3.h>
#include <CGAL/Triangulation_cell_base_with_info_3.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/intersections.h>

using namespace std;
using namespace __gnu_cxx;
#define HEURISTIC_K 2

namespace ORB_SLAM2
{

inline float SQ(float a){
    return a*a;
}

struct myPoint3f{
    float x, y, z;
};
struct myTriangles{
    myPoint3f p0, p1, p2 ;
};

class Delaunay3CellInfo;
//static list<Delaunay3CellInfo*> newAddedList;
//static list<int>newDeletedList ;
static int Delaunay3CellInfoID = 0 ;

class Delaunay3CellInfo
{
public:
    //static unordered_map<int, list<constraintInfo*>> attachConstraints2Cell;
    list<int> constrintIDList;
    int m_nMaxConstraintsKept;
    int id ;
    bool m_bNew;
    bool empty;

    // Constructors (it must be default-constructable)
    Delaunay3CellInfo()
    {
        constrintIDList.clear();
        id = Delaunay3CellInfoID++ ;
        m_bNew = true;
        m_nMaxConstraintsKept = HEURISTIC_K;
    }
    ~Delaunay3CellInfo()
    {
        //newDeletedList.push_back( id ) ;
    }
    Delaunay3CellInfo(const Delaunay3CellInfo & ref)
    {
        m_bNew = ref.m_bNew ;
        constrintIDList = ref.constrintIDList ;
        m_nMaxConstraintsKept = ref.m_nMaxConstraintsKept;
        id = ref.id ;
    }
    // Operators (It must be assignable)
    Delaunay3CellInfo & operator=(const Delaunay3CellInfo & rhs)
    {
        if (this != & rhs)
        {
            m_bNew = rhs.m_bNew ;
            constrintIDList = rhs.constrintIDList ;
            m_nMaxConstraintsKept = rhs.m_nMaxConstraintsKept;
        }
        return *this;
    }
    int getVoteCount() const {
        return constrintIDList.size();
    }
    bool isNew() const {
        return m_bNew;
    }
    bool isKeptByVoteCount(const int nVoteThresh = 1) const {
        if ( (int)constrintIDList.size() < nVoteThresh) {
            return true;
        }
        else{
            return false;
        }
    }
    void clearIntersections() {
        constrintIDList.clear();
    }
    void markOld() {
        m_bNew = false;
    }

    bool addIntersection(int constraintID)
    {
        if ( (int)constrintIDList.size() < HEURISTIC_K )
        {
            bool flag = true ;
            for( auto id : constrintIDList ){
                if ( id == constraintID ){
                    flag = false ;
                    break ;
                }
            }
            if ( flag ){
                constrintIDList.push_back(constraintID);
            }
            return flag ;
        }
        else {
            return false ;
        }
    }
};

// CGAL-related typedefs
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point;
typedef K::Triangle_3 Triangle;
typedef K::Segment_3 Segment;

// CGAL-related typedefs for Delaunay triangulation, 3-D
typedef CGAL::Triangulation_vertex_base_3<K> Vb;
typedef CGAL::Triangulation_hierarchy_vertex_base_3<Vb> Vbh;
typedef CGAL::Triangulation_cell_base_with_info_3<Delaunay3CellInfo, K> Cb;
typedef CGAL::Triangulation_data_structure_3<Vbh, Cb> Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds> Dt;
typedef CGAL::Triangulation_hierarchy_3<Dt> Delaunay3;
typedef Delaunay3::Point PointD3;

// Graph-cuts related typedefs
// typedef Graph<double, double, double> Graph_t; // Boykov & Kolmogorov's Code: TODO: implement a GraphWrapper for this.
typedef dlovi::GraphWrapper_Boost Graph_t; // Boykov & Kolmogorov's Code

struct collectedPoint
{
    int id ;
    float x, y, z ;
    bool flag ;
};

// define structure
struct pointInfo
{
    float p3d_old_x, p3d_old_y, p3d_old_z;
    float p3d_new_x, p3d_new_y, p3d_new_z;
    bool updateFlag ;
    int vertexID ;
    Delaunay3::Vertex_handle handle ;
    std::list<int> constraintIDList ;
    pointInfo(){
        constraintIDList.clear();
    }
    void check( float threshold = 0.0 )
    {
        float dx = p3d_old_x - p3d_new_x ;
        float dy = p3d_old_y - p3d_new_y ;
        float dz = p3d_old_z - p3d_new_z ;
        updateFlag = ( sqrt(dx*dx+dy*dy+dz*dz) > threshold) ;
    }
    bool updateInfo()
    {
        if ( updateFlag == false ){
            return false ;
        }
        else {
            p3d_old_x = p3d_new_x ;
            p3d_old_y = p3d_new_y ;
            p3d_old_z = p3d_new_z ;
            updateFlag = false ;
            return true ;
        }
    }
};

enum ConstraintType {
    CON_KF=0,
    CON_NONKF=1,
    CON_INFINITY=2
};

struct constraintInfo
{
    int constraintID, vertexID ;
    KeyFrame* pose ;
    bool updateFlag ;
    unsigned char kind ;
    float time ;
    float v3d_old_x, v3d_old_y, v3d_old_z;
    float pose3d_old_x, pose3d_old_y, pose3d_old_z;
    list<Delaunay3CellInfo*> listCell ;
    constraintInfo(){
        listCell.clear();
    }
    //    bool updateInfo()
    //    {
    //        if ( updateFlag == false ){
    //            return false ;
    //        }
    //        else {
    //            v3d_old_x = p3d_new_x ;
    //            v3d_old_y = p3d_new_y ;
    //            v3d_old_z = p3d_new_z ;
    //            updateFlag = false ;
    //            return true ;
    //        }
    //    }
};

struct localFrameInfo
{
    std::vector<int> vertexID ;
    double delta_T[4][4] ;
    float time ;
    KeyFrame* referenceKF ;
};

class FreespaceDelaunayAlgorithm
{
public:
    Delaunay3 dt;
    std::unordered_map<int, pointInfo> mapPointID2Info ;
    std::unordered_map<int, constraintInfo> mapConstraintID2Info ;
    int constraintNextID ;
    float mGlobalBATime ;

    // Hashing-related structs:
    struct HashVertHandle{
        size_t operator()(const Delaunay3::Vertex_handle x) const{
            return (size_t)(&(*x));
        } // use pointer to create hash
    };
    struct EqVertHandle{
        bool operator()(const Delaunay3::Vertex_handle x,
                        const Delaunay3::Vertex_handle y) const{
            return x == y;
        }
    };
    FreespaceDelaunayAlgorithm();
    void reset();
    void createBounds() ;

    void addSetOfConstraints(set<int>& setUnionedConstraints ) ;
    void addConstraintKF(int vertexID, KeyFrame *kF) ;
    void addConstraintNonKF( int vertexID, float pose_x, float pose_y, float pose_z, float time ) ;
    void removeConstraint( int constraintID ) ;
    void moveConstraint( int constraintID );

    void markTetrahedraCrossingConstraintWithBookKeeping(const Delaunay3::Vertex_handle& hndlQ,
                                                         const Segment &constraint,
                                                         int constraintID);
    void removeVertexSet(list<int>& currentOutlierVertexList , std::set<int> &setUnionedConstraints);
    void removeVertex_origin( int vertexID ) ;
    void addVertex(int vertexID, float x, float y, float z, set<int> &setUnionedConstraints);
    void moveVertexSet( const list<int> &moveVertexList );
    void moveVertex( int vertexID );

    void edgesSet(vector<pair<float, float> > &edges );

    void updateVertexAndConstraints(list<collectedPoint>& updatedPointsAndConstraints, KeyFrame *kF) ;
    void facetToTri(const Delaunay3::Facet & f, vector<Delaunay3::Vertex_handle> & vecTri);
    void tetsToTris(vector<dlovi::Matrix> & points, list<dlovi::Matrix> & tris);
    void tetsToTris_naive(vector<myPoint3f> & points, vector<myTriangles> &tris, const int nVoteThresh) ;
    void tetsToTris_naive2(vector<myPoint3f> & points, vector<myTriangles> &tris, vector<pair<myPoint3f, myPoint3f> > &edges, const int nVoteThresh) ;
    void tetsToTris_maxFlowSimple(vector<myPoint3f> &points, vector<myTriangles> &tris, const int nVoteThresh);
    void tetsToFreeSpace(vector<myTriangles> & tris);
    bool triangleConstraintIntersectionTest(const Delaunay3::Facet & tri,
                                            const dlovi::Matrix & segSrc,
                                            const dlovi::Matrix & segDest);
    bool triangleConstraintIntersectionTest(bool& bCrossesInteriorOfConstraint,
                                            const vector<dlovi::Matrix> & points,
                                            const dlovi::Matrix & tri,
                                            const pair<dlovi::Matrix, dlovi::Matrix> & constraint);
    bool cellTraversalExitTest(int & f, const Delaunay3::Cell_handle tetCur,
                               const Delaunay3::Cell_handle tetPrev, const dlovi::Matrix & matQ,
                               const dlovi::Matrix & matO);
};


class LocalMeshing
{
public:
    enum EntryType {
        ET_INVALID = -1,
        ET_RESET,
        ET_POINTINSERTION,
        ET_POINTDELETION,
        ET_POINTUPDATE,
        ET_VISIBILITYRAYINSERTION,
        ET_VISIBILITYRAYDELETION,
        ET_KEYFRAMEINSERTION,
        ET_KEYFRAMEDELETE,
        ET_KEYFRAMEUPDATE,
        ET_BUNDLEADJUSTMENT
    };

    LocalMeshing(const string &strSettingPath);
    ~LocalMeshing() ;
    bool mbFinished;
    long updatedLatestKFID ;
    double sumSpendTime ;
    int sumKF ;

    FreespaceDelaunayAlgorithm m_pAlgorithm;
    list<collectedPoint> updatedPointsAndConstraints;
    list<int> outlierVertexList ;
    std::mutex outlierVertexListQueue;
    vector<myPoint3f> points ;
    vector<myTriangles> tris ;
    vector<pair<myPoint3f,myPoint3f>> edges ;
    std::mutex mMutexMesh ;

    int img_width ;
    int img_height ;
    int renderingK ;
    int cvWaitTime ;
    float pointQualityThreshold ;
    float pointUpdateThreshold ;
    float constraintUpdateThreshold ;
    float distanceThreshold ;
    bool displayViewer ;
    cv::Mat curImg ;
    float curImgTime ;

    visualization_msgs::Marker meshMsg_, freeSpaceMsg_;
    FColorMap colorMap;

    cv::Mat M1l,M2l,M1r,M2r;

    ros::Publisher pubMesh_, pubPose_, pubObometry_, pubFreeSpace_ ;
    tf::TransformBroadcaster pubTf_;

    // Main function
    void Run();
    void buildMesh() ;
    void buildMesh2() ;
    void updateLocalVisibilityConstraints();
    void update(int state);
    void InsertKeyFrame(KeyFrame *pKF);
    bool reconstructFrameMesh(KeyFrame *pKF,
                              std::map<std::tuple<float, float, float>, KeyFrame *> &mapPointCorrelation,
                              int frameNum, Eigen::Vector3d centerT);
    void computePubPointAndColor( float x, float y, float z, const Eigen::Vector3d& currentT,
                                  geometry_msgs::Point& p, std_msgs::ColorRGBA& color_p );
    void updatePubPointList(const cv::Point3f& p3, const Eigen::Vector3d &currentT,
                            std::vector<geometry_msgs::Point> &pointList,
                            std::vector<std_msgs::ColorRGBA> &colorList);
    void checkChange(KeyFrame* mpCurrentKF);
    void removePoseInDt(KeyFrame* mpCurrentKF);
    void findKnearestKF(KeyFrame* mpCurrentKF, list<KeyFrame*>& nearestKF );

    std::list<KeyFrame*> mlpKeyFrameQueue;
    std::mutex mMutexKeyFrameQueue;
    std::vector<localFrameInfo> mLocalFrameInfoUnion ;
    std::list<int> mLocalFrameInfoQueue ;
    std::mutex mMutexLocalFrameInfoQueue ;

    bool displayDelaunyResult = true ;
    bool displayCenterFrame = true ;
    bool displayUnOptimizedPoints = true ;
    int meshSubdivNum = 0 ;

    bool onView = false ;
};

} //namespace ORB_SLAM

#endif // LOCALMESHING_H
