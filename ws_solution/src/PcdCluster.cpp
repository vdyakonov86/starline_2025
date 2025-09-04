#include "PcdCluster.hpp"


PcdCluster::PcdCluster() {}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
PcdCluster::cluster(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
  float tolerance,
  int min_size,
  int max_size
) {
  // Создаём kdtree для поиска соседей
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(tolerance);   // расстояние в метрах между точками в кластере
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  for (const auto& indices : cluster_indices)
  {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      for (int idx : indices.indices)
          cluster->push_back((*cloud)[idx]);

      cluster->width = cluster->size();
      cluster->height = 1;
      cluster->is_dense = true;

      clusters.push_back(cluster);
  }

  return clusters;
}
