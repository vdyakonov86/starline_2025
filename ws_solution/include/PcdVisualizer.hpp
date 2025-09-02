#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <chrono>
#include <thread>

enum class DisplayMode { Default, Height, Intensity };

class PcdVisualizer
{
private:
    pcl::visualization::PCLVisualizer::Ptr viewer;

public:
    PcdVisualizer();
    void showCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, DisplayMode mode);
    void showClouds(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds);
    void clear();
    void spin();
};