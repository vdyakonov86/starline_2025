#include "PcdLoader.hpp"
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_pcd>" << std::endl;
        return -1;
    }

    PcdLoader loader;
    if (!loader.load(argv[1]))
    {
        return -1;
    }

    auto cloud = loader.get();

    // Визуализация
    pcl::visualization::PCLVisualizer viewer("PCD Viewer");
    viewer.addPointCloud<pcl::PointXYZI>(cloud, "cloud");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return 0;
}
