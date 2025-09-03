
#include "PcdAnalyzer.hpp"

PcdAnalyzer::PcdAnalyzer() {}

ClusterFeatures
PcdAnalyzer::analyzePca(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
    ClusterFeatures features;

    // 1. PCA
    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(cloud);

    features.eigenvalues  = pca.getEigenValues();
    features.eigenvectors = pca.getEigenVectors();

    // 2. Размеры кластера по главным осям
    Eigen::Matrix3f eigVecs = pca.getEigenVectors();
    Eigen::Vector3f mean = pca.getMean().head<3>();

    Eigen::Vector3f minProj(FLT_MAX, FLT_MAX, FLT_MAX);
    Eigen::Vector3f maxProj(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (const auto& pt : cloud->points) {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        Eigen::Vector3f proj = eigVecs.transpose() * (p - mean);
        minProj = minProj.cwiseMin(proj);
        maxProj = maxProj.cwiseMax(proj);
    }

    features.sizes = maxProj - minProj;
    return features;
}

// Проверка "крестик или нет"
bool PcdAnalyzer::isCross(const ClusterFeatures& f,
             float expected_height,
             float expected_width,
             float expected_thickness,
             float tol) {
    float h = f.sizes[0];
    float w = f.sizes[1];
    float t = f.sizes[2];

    return (fabs(h - expected_height) / expected_height < tol) &&
           (fabs(w - expected_width) / expected_width < tol) &&
           (t < expected_thickness * (1.0 + tol));
}
