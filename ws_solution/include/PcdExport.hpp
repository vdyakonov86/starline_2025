#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class PcdExport
{
public:
    PcdExport();
    void exportPcd(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string &file_name);
};
