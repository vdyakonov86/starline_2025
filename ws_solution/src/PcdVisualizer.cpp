#include "PcdVisualizer.hpp"

PcdVisualizer::PcdVisualizer()
{
    viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
}

void PcdVisualizer::showCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, DisplayMode mode)
{
    viewer->removeAllPointClouds();

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity(cloud, "intensity");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> height(cloud, "z");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> white(cloud, 255, 255, 255);

    switch(mode)
    {
        case DisplayMode::Intensity:
            if (intensity.isCapable())
                viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity, "cloud");
            else
                viewer->addPointCloud<pcl::PointXYZI>(cloud, white, "cloud");
            break;

        case DisplayMode::Height:
            if (height.isCapable())
                viewer->addPointCloud<pcl::PointXYZI>(cloud, height, "cloud");
            else
                viewer->addPointCloud<pcl::PointXYZI>(cloud, white, "cloud");
            break;

        case DisplayMode::Default:
        default:
            viewer->addPointCloud<pcl::PointXYZI>(cloud, white, "cloud");
            break;
    }

    viewer->resetCamera();
}

void PcdVisualizer::spin()
{
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}