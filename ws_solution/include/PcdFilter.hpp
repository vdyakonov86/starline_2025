#pragma once

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PcdFilter
{
public:
    PcdFilter(float min_intensity, float max_intensity);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filter(
        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud);

private:
    float min_intensity_;
    float max_intensity_;
};
