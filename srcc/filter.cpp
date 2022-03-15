//
// Created by cch on 2022/3/10.
//

#include "../include/filter.h"


const pcl::PointCloud<pcl::PointXYZI>::Ptr Filter::readfromfile(const std::string path) {

   const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if(pcl::io::loadPCDFile(path,*cloud))//打开点云文件
    {
        std::cerr<<"error "<<std::endl;
        return nullptr;
    }
    return cloud;

}


pcl::PointIndicesPtr ground(new pcl::PointIndices);
pcl::PointCloud<pcl::PointXYZI>::Ptr Filter::my_pmf (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in) {

    // Compute the series of window sizes and height thresholds
    std::vector<float> height_thresholds;
    std::vector<float> window_sizes;   // window size
    std::vector<int> ground_indices;   //ground indices
    float iteration = 0.0;                 //number of iteration
    float window_size = 0.0f;
    float height_threshold = 0.0f;      //height threshold

    while (window_size < max_window_size) {
        // Determine the initial window size.
        if (exponential)
            window_size = cell_size * (2.0f * std::pow(2.0f, iteration) + 1.0f);
        else
            window_size = cell_size * (2.0f * (iteration + 1) * 2.0f + 1.0f);
        std::cout << "window_size  " << window_size << std::endl;
        // Calculate the height threshold to be used in the next iteration.
        if (iteration == 0)
            height_threshold = initial_distance;
        else
            height_threshold = slope * (window_size - window_sizes[iteration - 1]) * cell_size + initial_distance;
        std::cout << "height_threshold  " << height_threshold << std::endl;

        // Enforce max distance on height threshold
        if (height_threshold > max_distance)
            height_threshold = max_distance;

        window_sizes.push_back(window_size);
        height_thresholds.push_back(height_threshold);

        iteration++;
    }
    for (int i=0;i<cloud_in->points.size();i++)
    {
        ground_indices.push_back(i);
    }

    // Progressively filter ground returns using morphological open  window_sizes.size 为迭代次数
    for (size_t i = 0; i < window_sizes.size (); ++i)
    {
        std::cout<< "\nIteration " << i << " height threshold = " << height_thresholds[i]
                 << " window size = " <<window_sizes[i] << std::endl;

        // Limit filtering to those points currently considered ground returns
        typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud<pcl::PointXYZI> (*cloud_in, ground_indices, *cloud); //将cloud_in的索引为groung_indices的点云复制到cloud

        // Create new cloud to hold the filtered results. Apply the morphological
        // opening operation at the current window size.
        typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::applyMorphologicalOperator<pcl::PointXYZI> (cloud, window_sizes[i], pcl::MORPH_OPEN, *cloud_f);

        // Find indices of the points whose difference between the source and
        // filtered point clouds is less than the current height threshold.
        std::vector<int> pt_indices;
        //cout << "ground.size() = " << ground.size() << endl;
        for (size_t p_idx = 0; p_idx < ground_indices.size (); ++p_idx)
        {
            float diff = cloud->points[p_idx].z - cloud_f->points[p_idx].z;
            if (diff < height_thresholds[i])
                pt_indices.push_back (ground_indices[p_idx]);
        }

        // Ground is now limited to pt_indices
        ground_indices.swap (pt_indices);
        std::cout << "ground now has " << ground_indices.size () << " points" << std::endl;
    }
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZI>);
    // Extract cloud_in with ground indices
    pcl::copyPointCloud<pcl::PointXYZI> (*cloud_in, ground_indices, *cloud_out);
    ground->indices=ground_indices;   //索引赋值
    return cloud_out;
}
    void Filter::create(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in,pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

        std::cout << "Cloud before filtering: " << std::endl;
        std::cout << *cloud_in << std::endl;

        cloud_filtered = my_pmf(cloud_in);// ！！！Run progressive morphological filter

        std::cout << "\nGround cloud after filtering: " << std::endl;
        pcl::PCDWriter writer;

        // Extract non-ground returns
        pcl::ExtractIndices<pcl::PointXYZI> extract;  //extracts a set of indices from a point cloud
        extract.setInputCloud (cloud_in);
        extract.setIndices (ground);
        extract.setNegative (true);
        extract.filter (*cloud_filtered);

        *cloud_out = *cloud_filtered;

    }

 void Filter::ransac_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in,pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZI>);   //物体点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);//地面点云

        //创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // 创建分割对象
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        // 可选择配置，设置模型系数需要优化,建议打开
        seg.setOptimizeCoefficients(optimize);
        // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
        seg.setModelType(pcl::SACMODEL_PLANE);//设置模型类型
        seg.setMethodType(pcl::SAC_RANSAC);//设置随机采样一致性方法类型
        seg.setMaxIterations(maxIterations);//设置最大迭代次数
        seg.setDistanceThreshold(distanceThreshold);//设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
        seg.setInputCloud(cloud_in);
        //引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.empty()) {
            std::cout << "error! Could not found any inliers!" << std::endl;
        }
        // 从点云中抽取分割的处在平面上的点集
        pcl::ExtractIndices<pcl::PointXYZI> extractor;//点提取对象
        extractor.setInputCloud(cloud_in);
        extractor.setIndices(inliers);
        //true表示的是输入点集以外的点
        extractor.setNegative(true);
        extractor.filter(*obj_cloud);
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZI>("object.pcd", *obj_cloud, false);
        // 从点云中抽取分割的处在平面上的点集
        pcl::ExtractIndices<pcl::PointXYZI> extractor1;//点提取对象
        extractor1.setInputCloud(cloud_in);
        extractor1.setIndices(inliers);
        //false表示的是输入点集的点
        extractor1.setNegative(false);
        extractor1.filter(*ground_cloud);
        std::cout << "filter done." << std::endl;

        *cloud_out = *obj_cloud;
}

