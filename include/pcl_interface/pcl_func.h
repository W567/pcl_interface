#pragma once

#include "cloud_type.h"
#include <Eigen/Core>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/common/copy_point.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>


template<typename T>
void
downsample(
    const typename pcl::PointCloud<T>::Ptr &input,
    const typename pcl::PointCloud<T>::Ptr &output,
    const float voxel_);

template<typename T>
void
pass(
  const typename pcl::PointCloud<T>::Ptr &input,
  const typename pcl::PointCloud<T>::Ptr &output,
  const float min,
  const float max,
  const std::string axis,
  const bool state=false);

template<typename T>
void
estimateNormal(
  const typename pcl::PointCloud<T>::Ptr& input,
  const pcl::PointCloud<pcl::Normal>::Ptr& output_nor,
  const int rangeK);

template<typename T>
void
estimateNormalOMP(
    const typename pcl::PointCloud<T>::Ptr& input,
    const pcl::PointCloud<pcl::Normal>::Ptr& output_nor,
    const int rangeK);

template<typename T>
void
estimateNormalIntegral(
    const typename pcl::PointCloud<T>::Ptr& input,
    const pcl::PointCloud<pcl::Normal>::Ptr& output_nor,
    const float depth_change_factor,
    const float normal_smoothing_size);

template<typename T>
void
segmentPlane(
  const typename pcl::PointCloud<T>::Ptr& input,
  const typename pcl::PointCloud<pcl::Normal>::Ptr& input_nor,
  const pcl::ModelCoefficients::Ptr& coefficients_plane,
  const pcl::PointIndices::Ptr& inliers_plane,
  const float threshold,
  const int max_iterations=1000);

template<typename T>
void
extractPlane(
    const typename pcl::PointCloud<T>::Ptr& input,
    const typename pcl::PointCloud<T>::Ptr& output,
    float thre);

template<typename T>
void
extractPlane(
    const typename pcl::PointCloud<T>::Ptr& input,
    const typename pcl::PointCloud<T>::Ptr& output,
    const typename pcl::PointCloud<T>::Ptr &plane,
    float thre);

template<typename T>
void
extractCloud(
  const typename pcl::PointCloud<T>::Ptr& input,
  const typename pcl::PointIndices::Ptr inliers,
  const typename pcl::PointCloud<T>::Ptr& output,
  const bool state);

template<typename T>
void
removeOutlier(
  const typename pcl::PointCloud<T>::Ptr& input,
  const typename pcl::PointCloud<T>::Ptr& output,
  const int meanK,
  const float thresh);

template<typename T>
int
extractClusters(
  const typename pcl::PointCloud<T>::Ptr& input,
  std::vector<typename pcl::PointCloud<T>::Ptr>& clusters,
  const float tolerance,
  const int max_size,
  const int min_size);

template<typename T, typename pT>
int
obtainNearest(
  const typename pcl::KdTreeFLANN<T>::Ptr tree,
  const pT* spoint,
  float &dist);

template<typename T>
bool
icpCloud(
    const typename pcl::PointCloud<T>::Ptr target,
    const typename pcl::PointCloud<T>::Ptr input,
    const int max_iter=50,
    const float max_dist=0.01f,
    const float score_thre=0.0001f);

template<typename T>
void
PCACloud(
    const typename pcl::PointCloud<T>::Ptr input,
    const typename pcl::PointCloud<T>::Ptr output);

template<typename T>
void
proj2XOZ(
    const typename pcl::PointCloud<T>::Ptr input,
    const typename pcl::PointCloud<T>::Ptr output);

template <typename T>
void
widthXY(const typename pcl::PointCloud<T>::Ptr input, float& width_x, float& width_y);

template <typename T>
float
minZ(const typename pcl::PointCloud<T>::Ptr input);

template<typename T>
float
computeHeight(
  const typename pcl::PointCloud<T>::Ptr input);

template<typename T>
void
computeCentroid(
  const typename pcl::PointCloud<T>::Ptr input,
  Eigen::Vector3d &centroid);

template<typename T>
void
computeMaxR(
  const typename pcl::PointCloud<T>::Ptr input,
  const Eigen::Vector3d &centroid,
  float &max_r);

#include "impl/pcl_func.hpp"

