#include <iostream>

#include "PcdLoader.hpp"
#include "PcdVisualizer.hpp"
#include "PcdFilter.hpp"
#include "PcdCluster.hpp"
#include <pcl/common/common.h>

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
filter_by_geometry(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds) {
    const float H = 170 / 1000; // [м]
    const float W = H;
    const float T = 50 / 1000; 
    const float epsilon = 0.05;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> filtered;

    for (const auto& cloud : clouds) {
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        float dx = max_pt.x() - min_pt.x();
        float dy = max_pt.y() - min_pt.y();
        float dz = max_pt.z() - min_pt.z();

        // if (fabs(dx - W) < epsilon && fabs(dy - W) < epsilon && fabs(dz - H) < epsilon)
        if (fabs(dx - H) < epsilon && fabs(dy - W) < epsilon)
        {
            filtered.push_back(cloud);
        }
    }

    return filtered;
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_pcd>" << std::endl;
        return -1;
    }

    // Pcd load
    PcdLoader loader;
    if (!loader.load(argv[1]))
    {
        return -1;
    }

    auto cloud = loader.get();

    // Filter cloud by intensity
    PcdFilter pcdFilter(165, 255);
    auto cloud_intencity_filtered = pcdFilter.filter(cloud);

    // Clusterization
    PcdCluster pcdCluster;
    auto cloud_clusters = pcdCluster.cluster(cloud_intencity_filtered);

    // Filter cloud by cross geometry
    auto cloud_clusters_geometry_filtered = filter_by_geometry(cloud_clusters);

    // Visualization
    PcdVisualizer visualizer;
    // visualizer.showCloud(cloud, DisplayMode::Intensity);
    // visualizer.showCloud(cloud_intencity_filtered, DisplayMode::Default);

    visualizer.showClouds(cloud_clusters_geometry_filtered);
    visualizer.spin();

    return 0;
}
