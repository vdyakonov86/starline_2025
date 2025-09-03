
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

bool PcdAnalyzer::isCross(
    const ClusterFeatures& f,
    float expected_height,
    float expected_width,
    float max_thickness,
    float tol)
{
    float size_max = f.sizes[0]; // высота крестика
    float size_mid = f.sizes[1]; // ширина бруса
    float size_min = f.sizes[2]; // толщина

    // 1. Проверка "плоский объект"
    if (size_min > max_thickness) {
        return false;
    }

    // 2. Проверка размеров на соответствие эталону (с допуском tol)
    auto within_tol = [tol](float value, float expected) {
        return std::fabs(value - expected) / expected <= tol;
    };

    if (!within_tol(size_max, expected_height)) return false;
    if (!within_tol(size_mid, expected_width)) return false;

    // 3. Проверка симметрии H и W (чтобы отбросить прямоугольники)
    float ratio = size_max / size_mid;
    if (ratio > 1.0f + tol) {
        return false;
    }

    return true;
}

