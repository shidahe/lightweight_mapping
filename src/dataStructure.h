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

#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <algorithm>
#include <vector>

class myVector2d
{
public:
    double x_;
    double y_;

    myVector2d(double x, double y):x_(x), y_(y){}
    myVector2d():x_(0), y_(0){}

    double CrossProduct(const myVector2d& vec)
    {
        return x_*vec.y_ - y_*vec.x_;
    }
};

class myVector2f
{
public:
    float x_;
    float y_;

    myVector2f(float x, float y):x_(x), y_(y){}
    myVector2f():x_(0), y_(0){}

    float CrossProduct(const myVector2f& vec)
    {
        return x_*vec.y_ - y_*vec.x_;
    }
};

struct my_point2{
    float x, y ;
    bool operator <(const my_point2& b)const{
        if ( y != b.y ) return y < b.y ;
        return x < b.x ;
    }
};

inline bool IsPointInTriangle(myVector2d& PA, myVector2d& PB, myVector2d& PC)
{
    double t1 = PA.CrossProduct(PB);
    double t2 = PB.CrossProduct(PC);
    double t3 = PC.CrossProduct(PA);
    return t1*t2 >= 0 && t1*t3 >= 0;
}

class CONVEX_HULL
{
public:
    CONVEX_HULL()
    {
        ;
    }
    ~CONVEX_HULL(){
        ;
    }

    std::vector<my_point2> pList ;
    std::vector<my_point2> corners ;

    float dot( my_point2 & a , my_point2 &b , my_point2 &c ){
        float x1 = b.x - a.x , y1 = b.y - a.y ;
        float x2 = c.x - a.x , y2 = c.y - a.y ;
        return x1 * y2 - y1 * x2 ;
    }

    void insert( float x, float y)
    {
        my_point2 tmp ;
        tmp.x = x ;
        tmp.y = y ;
        pList.push_back(tmp) ;
    }

    bool insideHull( float x, float y)
    {
        if ( corners.size() < 3 ){
            return false ;
        }
        bool flag = true ;
        for( int i=0, sz = corners.size(); i < sz; i++ )
        {
            int j = (i+1)%sz ;
            int k = (i+2)%sz ;
            myVector2f PA( corners[i].x - x, corners[i].y - y ) ;
            myVector2f PB( corners[j].x - x, corners[j].y - y ) ;
            myVector2f PC( corners[k].x - x, corners[k].y - y ) ;

            float t1 = PA.CrossProduct(PB);
            float t2 = PB.CrossProduct(PC);
            if ( t1*t2 >= 0 ){
            }
            else {
                flag = false ;
                break ;
            }
        }
        return flag ;
    }

    void graham_scan()
    {
        int i , temp ;
        int top = 0 ;
        int n = pList.size();

        std::sort( pList.begin(), pList.end() ) ;

        std::vector<int>stack( pList.size()+5 );
        stack[top] = 0 , stack[++top] = 1 ;
        for ( i = 2 ; i < n ; ++i )
        {
            while ( top >= 1 && dot( pList[stack[top-1]] , pList[stack[top]] , pList[i] ) <= 0 ) --top ;
            stack[++top] = i ;
        }
        temp = top ;

        stack[++top] = n - 2 ;
        for ( i = n - 3 ; i >= 0 ; --i )
        {
            while ( top >= temp + 1 && dot( pList[stack[top-1]] , pList[stack[top]] , pList[i] ) <= 0 ) --top ;
            stack[++top] = i ;
        }
        corners.clear();
        corners.resize(top);
        for( i=0; i < top ; i++ ){
            //printf("i=%d stack[i]=%d pList.sz=%d corners.sz=%d\n", i, stack[i], pList.size(), corners.size() ) ;
            corners[i] = pList[stack[i]] ;
        }
    }
};

struct attachPoints{
    float u, v, depth ;
};

inline void calculateDepthImage(cv::Mat disparity, cv::Mat& depthImage, float bf)
{
    int n = disparity.rows ;
    int m = disparity.cols ;

    depthImage.create(n, m, CV_32F);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            float d = disparity.at<float>(i, j);
            if ( d < 3.0 ) {
                depthImage.at<float>(i, j) = 0.0;
            }
            else {
                depthImage.at<float>(i, j) = bf / d;
            }
        }
    }
}

inline Eigen::Matrix3d MakeMatrix( Eigen::Vector3d& X, Eigen::Vector3d& Y )
{
    // make sure that we actually have two unique vectors.
    assert( X != Y );

    Eigen::Matrix3d R;
    Eigen::Vector3d b0 = X ;
    b0.normalize() ;
    Eigen::Vector3d b2 = X.cross(Y) ;
    b2.normalize() ;
    Eigen::Vector3d b1 = b2.cross(X) ;
    b1.normalize() ;

    R.col(0) = b0;
    R.col(1) = b1;
    R.col(2) = b2;

    return R.transpose();
}

#endif // DATASTRUCTURE_H
