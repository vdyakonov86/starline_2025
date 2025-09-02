#include "PcdExport.hpp"

PcdExport::PcdExport() {}

void PcdExport::exportPcd(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string &file_name) {
  if (pcl::io::savePCDFileBinary(file_name, *cloud) == 0)
  {
      std::cout << "Saved cloud to " << file_name << "\n";
  }
  else
  {
      std::cerr << "Error saving cloud!\n";
  }
}
