//
//  main.cpp
//  rgb-slam
//
//  Created by sofiawu on 2017/7/10.
//  Copyright © 2017年 sofiawu. All rights reserved.
//

#include<iostream>
#include "slamBase.hpp"
using namespace std;

#include <opencv2/core/eigen.hpp>

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

// Eigen !
#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, const char * argv[]) {
    //本节要拼合data中的两对图像
    ParameterReader pd("/Users/sofiawu/GitHub/rgbd-slam-test/data/parameters.txt");
    
    // 声明两个帧，FRAME结构请见include/slamBase.h
    FRAME frame1, frame2;
    
    //读取图像
    frame1.rgb = cv::imread( "/Users/sofiawu/GitHub/rgbd-slam-test/data/jointpointcloud/rgb1.png" );
    frame1.depth = cv::imread( "/Users/sofiawu/GitHub/rgbd-slam-test/data/jointpointcloud/depth1.png", -1);
    frame2.rgb = cv::imread( "/Users/sofiawu/GitHub/rgbd-slam-test/data/jointpointcloud/rgb2.png" );
    frame2.depth = cv::imread( "/Users/sofiawu/GitHub/rgbd-slam-test/data/jointpointcloud/depth2.png", -1 );
    
    // 提取特征并计算描述子
    cout<<"extracting features"<<endl;
    
    computeKeyPointsAndDesp( frame1 );
    computeKeyPointsAndDesp( frame2 );
    
    // 相机内参
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );
    
    cout<<"solving pnp"<<endl;
    // 求解pnp
    RESULT_OF_PNP result = estimateMotion( frame1, frame2, camera );
    
    cout<<result.rvec<<endl<<result.tvec<<endl;
    
    // 处理result
    // 将旋转向量转化为旋转矩阵
    cv::Mat R;
    cv::Rodrigues( result.rvec, R );
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);
    
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(r);
    cout<<"translation"<<endl;
    Eigen::Translation<double,3> trans(result.tvec.at<double>(0,0), result.tvec.at<double>(0,1), result.tvec.at<double>(0,2));
    T = angle;
    T(0,3) = result.tvec.at<double>(0,0);
    T(1,3) = result.tvec.at<double>(0,1);
    T(2,3) = result.tvec.at<double>(0,2);
    
    // 转换点云
    cout<<"converting image to clouds"<<endl;
    PointCloud::Ptr cloud1 = image2PointCloud( frame1.rgb, frame1.depth, camera );
    PointCloud::Ptr cloud2 = image2PointCloud( frame2.rgb, frame2.depth, camera );
    
    // 合并点云
    cout<<"combining clouds"<<endl;
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *cloud1, *output, T.matrix() );
    *output += *cloud2;
    pcl::io::savePCDFile("/Users/sofiawu/GitHub/rgbd-slam-test/data/jointpointcloud/result.pcd", *output);
    cout<<"Final result saved."<<endl;
    
    /*pcl::visualization::PCLVisualizer viewer( "viewer" );
    viewer.addPointCloud<PointT> (output, "sample cloud");
    //viewer.showCloud( output );
    while( !viewer.wasStopped() )
    {
        viewer.spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }*/
    return 0;
}
