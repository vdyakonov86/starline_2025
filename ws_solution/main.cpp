#include <iostream>

#include "PcdLoader.hpp"
#include "PcdVisualizer.hpp"
#include "PcdFilter.hpp"
#include "PcdCluster.hpp"

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

    // Filtering
    PcdFilter pcdFilter(165, 255);
    auto cloud_intencity_filtered = pcdFilter.filter(cloud);

    // Clusterization
    PcdCluster pcdCluster;
    auto cloud_clusters = pcdCluster.cluster(cloud_intencity_filtered);

    // Visualization
    PcdVisualizer visualizer;
    // visualizer.showCloud(cloud, DisplayMode::Intensity);
    // visualizer.showCloud(cloud_intencity_filtered, DisplayMode::Default);

    visualizer.showClouds(cloud_clusters);
    visualizer.spin();

    return 0;
}
