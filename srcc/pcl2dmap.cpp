//
// Created by cch on 2022/3/9.
//

#include "../include/pcl2dmap.h"
using namespace rbk;


inline  double Solution::roundTo(double num, int digit) {

    if (digit > 37 || digit < -37) return num;
    double factor = std::pow(10, digit);
    return round(num * factor) / factor;
}

void Solution::save_smap(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double height, double resolution) {

    std::cout << "saving smap... cloud points size: " << cloud->points.size() << std::endl;
    // double angle = mapAngle(*mapData, mSLAM->GetMapConfigData()->resolution);
    rbk::protocol::Message_Map rbk_smap;
    double xmin = 9999999999.0;
    double ymin = 9999999999.0;
    double xmax = -9999999999.0;
    double ymax = -9999999999.0;
//passthrough filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> passthrough;
    passthrough.setInputCloud(cloud);//输入点云
    passthrough.setFilterFieldName("z");//对z轴进行操作
    //passthrough.setFilterFieldName("y");
    //passthrough.setFilterFieldName("y");
    passthrough.setFilterLimits(0.0, 400.0);//设置直通滤波器操作范围
    //passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
    passthrough.filter(*cloud_after_PassThrough);//执行滤波，过滤结果保存在 cloud_after_PassThrough


    for (auto point : cloud_after_PassThrough->points) {
        rbk::protocol::Message_MapPos *pos3d = rbk_smap.add_normal_pos3d_list();
        pos3d->set_x(roundTo(point.x, 3));
        pos3d->set_y(roundTo(point.y, 3));
        pos3d->set_z(roundTo(point.z, 3));
        if (point.z > -height && point.z < height) {
            rbk::protocol::Message_MapPos *pos = rbk_smap.add_normal_pos_list();
            pos->set_x(roundTo(point.x, 3));
            pos->set_y(roundTo(point.y, 3));
            xmax = std::max(double(point.x), xmax);
            xmin = std::min(double(point.x), xmin);
            ymin = std::min(double(point.y), ymin);
            ymax = std::max(double(point.y), ymax);
        }
    }
    std::cout << "3d pos size: " << rbk_smap.normal_pos3d_list_size() << std::endl;

    rbk::protocol::Message_MapHeader mapHeader;
    mapHeader.set_map_type("2D-Map");

    std::string out_file_name;
    boost::posix_time::ptime now(boost::posix_time::second_clock::local_time());
    auto *tf = new boost::posix_time::time_facet("%Y-%m-%d_%H-%M-%S");
    std::ostringstream oss;
    oss.imbue(std::locale(oss.getloc(), tf));
    oss << now;
    out_file_name.append(oss.str());
    mapHeader.set_map_name(out_file_name);
    mapHeader.set_resolution(resolution);
    rbk::protocol::Message_MapPos minPos, maxPos;
    minPos.set_x(roundTo(xmin, 3));
    minPos.set_y(roundTo(ymin, 3));
    maxPos.set_x(roundTo(xmax, 3));
    maxPos.set_y(roundTo(ymax, 3));
    mapHeader.mutable_min_pos()->CopyFrom(minPos);
    mapHeader.mutable_max_pos()->CopyFrom(maxPos);
    mapHeader.set_version("1.0.6");
    rbk_smap.mutable_header()->CopyFrom(mapHeader);
    out_file_name.append(".smap");
    std::string stringMap;

    google::protobuf::util::MessageToJsonString(rbk_smap, &stringMap);
    std::ofstream of;
    of.open(out_file_name);
    if (of.is_open()) {
        of << stringMap;
        of.close();
        std::cout << "smap saved! " << out_file_name;
    }
}