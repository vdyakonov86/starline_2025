#pragma once
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class PcdLoader
{
private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

public:
    PcdLoader();                                // конструктор по умолчанию
    bool load(const std::string& path);         // загрузить pcd
    pcl::PointCloud<pcl::PointXYZI>::Ptr get(); // получить облако
};
