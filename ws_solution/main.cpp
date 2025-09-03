#include <iostream>
#include <fstream>

#include "PcdLoader.hpp"
#include "PcdVisualizer.hpp"
#include "PcdFilter.hpp"
#include "PcdCluster.hpp"
#include "PcdAnalyzer.hpp"
#include "PcdExport.hpp"
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

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

    std::cout << "Clusterization" << "\n" << std::endl;
    PcdCluster pcdCluster;
    auto cloud_clusters = pcdCluster.cluster(cloud_intencity_filtered);
    std::cout << "cloud_clusters: " << cloud_clusters.size() << std::endl;


    std::cout << "Analyze PCA" << "\n" << std::endl;
    const float cross_h = 170 / 1000.0; 
    const float cross_w = 170 / 1000.0; 
    const float cross_thick = 0.05;
    const float tolerance = 0.2;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters_pca_filtered;
    PcdAnalyzer pcdAnalyzer;
    for (const auto &cld: cloud_clusters) {
        ClusterFeatures features = pcdAnalyzer.analyzePca(cld);
        bool is_cross = pcdAnalyzer.isCross(features, cross_h, cross_w, cross_thick, tolerance);

        if (is_cross) {
            cloud_clusters_pca_filtered.push_back(cld);
        }
    }
    std::cout << "cloud_clusters_pca_filtered: " << cloud_clusters_pca_filtered.size() << std::endl;

    std::vector<Eigen::Vector4f> centroids;
    std::vector<Eigen::Vector3f> centers;
    std::ofstream ofs("/starline/cross_centers.txt", std::ios::trunc);

    for (const auto &cluster : cloud_clusters_pca_filtered) {
        auto centroid = pcdAnalyzer.findCrossCentroid(cluster);
        centroids.push_back(centroid);

        Eigen::Vector3f center = centroid.head<3>();
        centers.push_back(center);

        ofs << centroid[0] << " " << centroid[1] << " " << centroid[2] << "\n";
    }
    ofs.close();

    std::cout << "Merge found clusters to one cloud: " << "\n" << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto &cld: cloud_clusters_pca_filtered) {
        *cloud_merged = *cloud_merged + *cld;
    }
   
    std::cout << "Visualization" << "\n" << std::endl;
    PcdVisualizer visualizer;
    // visualizer.showCloud(cloud, DisplayMode::Intensity);
    // visualizer.showCloud(cloud_intencity_filtered, DisplayMode::Default);
    // visualizer.showClouds(cloud_clusters_pca_filtered);

    for (size_t i = 0; i < cloud_clusters_pca_filtered.size(); ++i) {
        visualizer.showCloudWithCenter(cloud_clusters_pca_filtered[i], "cluster_" + std::to_string(i), centers[i]);
    }
    visualizer.spin();


    std::cout << "Export" << "\n" << std::endl;
    PcdExport pcdExport;
    pcdExport.exportPcd(cloud_merged, argv[2]);

    return 0;
}
