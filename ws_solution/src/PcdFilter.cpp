#include "PcdFilter.hpp"

PcdFilter::PcdFilter() {}

pcl::PointCloud<pcl::PointXYZI>::Ptr
PcdFilter::filterByIntensity(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud,
    const float &min,
    const float &max)
{
    auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    output_cloud->reserve(input_cloud->size());

    for (const auto& point : *input_cloud)
    {
        if (point.intensity >= min && point.intensity <= max)
            output_cloud->push_back(point);
    }

    output_cloud->width = static_cast<uint32_t>(output_cloud->size());
    output_cloud->height = 1;
    output_cloud->is_dense = true;

    return output_cloud;
}
