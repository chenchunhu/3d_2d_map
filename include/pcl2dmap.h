//
// Created by cch on 2022/3/9.
//

#ifndef INC_3D_2D_MAP_PCL2DMAP_H
#define INC_3D_2D_MAP_PCL2DMAP_H

#define BOOST_SYSTEM_NO_DEPRECATED

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/conversions.h>
#include <iostream>
#include <google/protobuf/util/json_util.h>
#include "message_map.pb.h"
#include<pcl/filters/passthrough.h>                  //直通滤波器头文件
#include<pcl/filters/voxel_grid.h>                   //体素滤波器头文件
#include<pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>          //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>       //半径滤波器头文件


class Solution
{
public:
    inline  double roundTo(double num, int digit);
    void save_smap(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double height, double resolution);

};


#endif //INC_3D_2D_MAP_PCL2DMAP_H
