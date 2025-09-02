#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

class PcdCluster 
{
public:
    PcdCluster();

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
    cluster(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
};