#include <iostream>

#include "PcdLoader.hpp"
#include "PcdVisualizer.hpp"
#include "PcdFilter.hpp"
#include "PcdCluster.hpp"
#include "PcdAnalyzer.hpp"
#include "PcdExport.hpp"
#include <pcl/common/common.h>

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
filter_by_geometry(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds) {
    const float H = 170 / 1000; // [м]
    const float T = 50 / 1000; 
    const float epsilon = 0.4;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> filtered;

    for (const auto& cloud : clouds) {
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        float dx = max_pt.x() - min_pt.x();
        float dy = max_pt.y() - min_pt.y();
        float dz = max_pt.z() - min_pt.z();

        if (fabs(dx - H) < epsilon && fabs(dy - H) < epsilon && fabs(dz - H) < epsilon)
        {
            std::cout << "dx: " << dx << " dy: " << dy << " dz: " << dz << std::endl;
            std::cout << "diff x: " << fabs(dx - H) << " diff y: " << fabs(dy - H)  << " diff z: " << fabs(dz - H)  << std::endl;
            filtered.push_back(cloud);
        }
    }

    return filtered;
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_pcd> <path_to_out_pcd>" << std::endl;
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
    PcdFilter pcdFilter(120, 255);
    auto cloud_intencity_filtered = pcdFilter.filter(cloud);

    std::cout << "Clusterization" << std::endl;
    PcdCluster pcdCluster;
    auto cloud_clusters = pcdCluster.cluster(cloud_intencity_filtered);
    std::cout << "cloud_clusters: " << cloud_clusters.size() << std::endl;

    // Filter cloud by cross geometry
    // auto cloud_clusters_geometry_filtered = filter_by_geometry(cloud_clusters);
    const float cross_h = 170 / 1000.0; 
    const float cross_w = 170 / 1000.0; 
    const float cross_thick = 0.05;
    const float tolerance = 0.2;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters_pca_filtered;
    std::cout << "Analyze PCA" << std::endl;
    PcdAnalyzer pcdAnalyzer;
    for (const auto &cld: cloud_clusters) {
        ClusterFeatures features = pcdAnalyzer.analyzePca(cld);
        bool is_cross = pcdAnalyzer.isCross(features, cross_h, cross_w, cross_thick, tolerance);

        if (is_cross) {
            cloud_clusters_pca_filtered.push_back(cld);
        }
    }

    std::cout << "cloud_clusters_pca_filtered: " << cloud_clusters_pca_filtered.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointXYZI>);

    for (const auto &cld: cloud_clusters_pca_filtered) {
        *cloud_merged = *cloud_merged + *cld;
    }

    // Export
    PcdExport pcdExport;
    pcdExport.exportPcd(cloud_merged, argv[2]);

    // Visualization
    PcdVisualizer visualizer;
    // visualizer.showCloud(cloud, DisplayMode::Intensity);
    // visualizer.showCloud(cloud_intencity_filtered, DisplayMode::Default);

    visualizer.showClouds(cloud_clusters_pca_filtered);
    visualizer.spin();

    return 0;
}
