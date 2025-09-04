#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

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
        std::cerr << "Usage: " << argv[0] << "<path_to_pcd> <path_to_config.yaml>" << std::endl;
        return -1;
    }

    // Pcd load
    PcdLoader loader;
    if (!loader.load(argv[1]))
    {
        return -1;
    }

    auto cloud = loader.get();
    auto path_to_config = std::string(argv[2]);
    YAML::Node config = YAML::LoadFile(path_to_config);

    // Parse config
    const float min_intensity = config["filter_intensity"]["min"].as<float>();
    const float max_intensity = config["filter_intensity"]["max"].as<float>();

    const float clust_tol = config["clustering"]["tolerance"].as<float>();
    const int clust_min_s = config["clustering"]["min_size"].as<int>();
    const int clust_max_s = config["clustering"]["max_size"].as<int>();

    const float cross_h = config["cross_params"]["height"].as<float>();
    const float cross_w = config["cross_params"]["width"].as<float>();
    const float cross_thick = config["cross_params"]["thickness"].as<float>();

    const float pca_tol = config["pca"]["tolerance"].as<float>();

    std::cout << "Filter cloud by intensity" << "\n" << std::endl;
    PcdFilter pcdFilter;
    auto cld_intensity_filtered = pcdFilter.filterByIntensity(cloud, min_intensity, max_intensity);

    std::cout << "Clusterization" << "\n" << std::endl;
    PcdCluster pcdCluster;
    auto clusters = pcdCluster.cluster(cld_intensity_filtered, clust_tol, clust_min_s, clust_max_s);
    std::cout << "clusters: " << clusters.size() << std::endl;

    std::cout << "Analyze PCA" << "\n" << std::endl;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters_pca_filtered;
    PcdAnalyzer pcdAnalyzer;
    for (const auto &cluster: clusters) {
        ClusterFeatures features = pcdAnalyzer.analyzePca(cluster);
        bool is_cross = pcdAnalyzer.isCross(features, cross_h, cross_w, cross_thick, pca_tol);

        if (is_cross) {
            clusters_pca_filtered.push_back(cluster);
        }
    }
    std::cout << "clusters_pca_filtered: " << clusters_pca_filtered.size() << std::endl;

    std::vector<Eigen::Vector4f> centroids;
    std::vector<Eigen::Vector3f> centers;
    std::ofstream ofs("/starline/cross_centers.txt", std::ios::trunc);

    for (const auto &cluster : clusters_pca_filtered) {
        auto centroid = pcdAnalyzer.findCrossCentroid(cluster);
        centroids.push_back(centroid);

        Eigen::Vector3f center = centroid.head<3>();
        centers.push_back(center);

        ofs << centroid[0] << " " << centroid[1] << " " << centroid[2] << "\n";
    }
    ofs.close();

    std::cout << "Merge found clusters to one cloud" << "\n" << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto &cld: clusters_pca_filtered) {
        *cloud_merged = *cloud_merged + *cld;
    }
   
    std::cout << "Visualization" << "\n" << std::endl;
    PcdVisualizer visualizer;
    // visualizer.showCloud(cloud, DisplayMode::Intensity);
    // visualizer.showCloud(cloud_intensity_filtered, DisplayMode::Default);
    // visualizer.showClouds(clusters_pca_filtered);

    for (size_t i = 0; i < clusters_pca_filtered.size(); ++i) {
        visualizer.showCloudWithCenter(clusters_pca_filtered[i], "cluster_" + std::to_string(i), centers[i]);
    }
    visualizer.spin();


    std::cout << "Export" << "\n" << std::endl;
    PcdExport pcdExport;
    pcdExport.exportPcd(cloud_merged, argv[2]);

    return 0;
}
