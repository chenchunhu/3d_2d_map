#include "pcl2dmap.h"
#include "filter.h"
#include <iostream>

const std::string path = "/home/cch/pcd/1.pcd";

int main() {

    auto * filter = new Filter;
    auto solution = new Solution;

    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = filter->readfromfile(path);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZI>);

    Filter::ransac_filter(cloud,cloud_filter);

    solution->save_smap(cloud_filter,2,2);

    return 0;
}

