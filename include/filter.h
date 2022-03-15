//
// Created by cch on 2022/3/10.
//

#ifndef INC_3D_2D_MAP_FILTER_H
#define INC_3D_2D_MAP_FILTER_H




#include "../include/pcl2dmap.h"
#include <pcl/filters/morphological_filter.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#define max_window_size 0.05f
#define slope 0.7f
#define max_distance 0.5f
#define initial_distance 0.25f
#define cell_size 0.01f
#define base 2.0f
#define exponential true

#define optimize true
#define maxIterations 20000
#define distanceThreshold 1.2

class Filter
{
public:
    //pcl::PointCloud<pcl::PointXYZI>::Ptr my_pmf (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in);
    //void create(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    static void  ransac_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in,pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out);
    const pcl::PointCloud<pcl::PointXYZI>::Ptr readfromfile(const std::string path);
    pcl::PointCloud<pcl::PointXYZI>::Ptr my_pmf(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in);
    void create(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in,pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out);

};
#endif //INC_3D_2D_MAP_FILTER_H