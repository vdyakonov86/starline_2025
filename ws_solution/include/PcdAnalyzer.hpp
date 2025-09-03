#include <cfloat>
#include <Eigen/Dense>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>

struct ClusterFeatures {
    Eigen::Vector3f eigenvalues;
    Eigen::Matrix3f eigenvectors;
    Eigen::Vector3f sizes; // длина проекции по каждой оси
};

class PcdAnalyzer {
  public:
    PcdAnalyzer();

    ClusterFeatures analyzePca(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
    // Проверка "крестик или нет"
    bool isCross(
      const ClusterFeatures& f,
      float expected_height,
      float expected_width,
      float max_thickness,
      float tol = 0.2);
};