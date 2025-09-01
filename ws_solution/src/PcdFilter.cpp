#include "PcdFilter.hpp"

PcdFilter::PcdFilter(float min_intensity, float max_intensity)
    : min_intensity_(min_intensity), max_intensity_(max_intensity) {}

pcl::PointCloud<pcl::PointXYZI>::Ptr
PcdFilter::filter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud)
{
    auto output_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    output_cloud->reserve(input_cloud->size());

    for (const auto& point : *input_cloud)
    {
        if (point.intensity >= min_intensity_ && point.intensity <= max_intensity_)
        {
            output_cloud->push_back(point);
        }
    }

    output_cloud->width = static_cast<uint32_t>(output_cloud->size());
    output_cloud->height = 1;
    output_cloud->is_dense = true;

    return output_cloud;
}
