#pragma once

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PcdFilter
{
public:
    PcdFilter();

    pcl::PointCloud<pcl::PointXYZI>::Ptr 
    filterByIntensity(
        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud,
        const float &min,
        const float &max);
};
