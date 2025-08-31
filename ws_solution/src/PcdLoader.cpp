#include "PcdLoader.hpp"

PcdLoader::PcdLoader()
    : cloud(new pcl::PointCloud<pcl::PointXYZI>) {}

bool PcdLoader::load(const std::string& path)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(path, *cloud) == -1)
    {
        std::cerr << "Couldn't read file " << path << std::endl;
        return false;
    }
    std::cout << "Loaded " << cloud->points.size() 
              << " points from " << path << std::endl;
    return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PcdLoader::get()
{
    return cloud;
}
