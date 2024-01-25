template<typename T>
void
downsample(
    const typename pcl::PointCloud<T>::Ptr &input,
    const typename pcl::PointCloud<T>::Ptr &output,
    const float voxel_)
{
  pcl::VoxelGrid<T> sor;
  sor.setInputCloud(input);
  sor.setLeafSize(voxel_, voxel_, voxel_);
  sor.filter(*output);
}

/*
  state: true: remove points in the range
         false: remove points out of the range (default)
*/
template<typename T>
void
pass(
    const typename pcl::PointCloud<T>::Ptr &input,
    const typename pcl::PointCloud<T>::Ptr &output,
    const float min, const float max, const std::string axis, const bool state)
{
  pcl::PassThrough<T> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName(axis);
  pass.setFilterLimits(min, max);
  pass.setNegative(state);
  pass.filter(*output);
}

/*
  passthrough filter with normal comparison
  points outside of the range or with normal intersection angle larger than nor_thre will be kept
*/
template<typename T>
void
passWithNormal(
    const typename pcl::PointCloud<T>::Ptr &input,
    const typename pcl::PointCloud<T>::Ptr &output,
    const Eigen::Vector3d &normal,
    const float min, const float max, const std::string axis, const float nor_thre)
{
  typename pcl::PointCloud<T>::Ptr tmp (new pcl::PointCloud<T>);
  if (axis == "x")
  {
    for (const auto& point : input->points)
    {
      if (point.x < min || point.x > max || abs(std::acos(inner(normal, point.normal))) > nor_thre)
      {
        tmp->points.push_back(point);
      }
    }
  }
  else if (axis == "y")
  {
    for (const auto& point : input->points)
    {
      if (point.y < min || point.y > max || abs(std::acos(inner(normal, point.normal))) > nor_thre)
      {
        tmp->points.push_back(point);
      }
    }
  }
  else if (axis == "z")
  {
    for (const auto& point : input->points)
    {
      if (point.z < min || point.z > max || abs(std::acos(inner(normal, point.normal))) > nor_thre)
      {
        tmp->points.push_back(point);
      }
    }
  }
  else
  {
    std::cout << "Error: axis must be x, y or z" << std::endl;
    return;
  }
  *output = *tmp;
}

/*
  setViewPoint (float vpx, float vpy, float vpz); can be used to set the viewpoint of the camera.
*/
template<typename T>
void
estimateNormal(
    const typename pcl::PointCloud<T>::Ptr& input,
    const pcl::PointCloud<pcl::Normal>::Ptr& output_nor,
    const int rangeK)
{
  pcl::NormalEstimation<T, pcl::Normal> ne;
  typename pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T> ());
  ne.setInputCloud(input);
  ne.setSearchMethod(tree);
  ne.setKSearch(rangeK);
  ne.compute(*output_nor);
}

template<typename T>
void
estimateNormalOMP(
    const typename pcl::PointCloud<T>::Ptr& input,
    const pcl::PointCloud<pcl::Normal>::Ptr& output_nor,
    const int rangeK)
{
  pcl::NormalEstimationOMP<T, pcl::Normal> ne;
  typename pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T> ());
  ne.setInputCloud(input);
  ne.setSearchMethod(tree);
  ne.setKSearch(rangeK);
  ne.compute(*output_nor);
}

template<typename T>
void
estimateNormalIntegral(
    const typename pcl::PointCloud<T>::Ptr& input,
    const pcl::PointCloud<pcl::Normal>::Ptr& output_nor,
    const float depth_change_factor,
    const float normal_smoothing_size)
{
  pcl::IntegralImageNormalEstimation<T, pcl::Normal> ne;
  ne.setInputCloud(input);
  // COVARIANCE_MATRIX, AVERAGE_DEPTH_CHANGE, AVERAGE_3D_GRADIENT, SIMPLE_3D_GRADIENT
  ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(depth_change_factor);
  ne.setNormalSmoothingSize(normal_smoothing_size);
  ne.compute(*output_nor);
}

template<typename T>
void
segmentPlane(
    const typename pcl::PointCloud<T>::Ptr& input,
    const typename pcl::PointCloud<pcl::Normal>::Ptr& input_nor,
    const pcl::ModelCoefficients::Ptr& coefficients_plane,
    const pcl::PointIndices::Ptr& inliers_plane,
    const float threshold,
    const int max_iterations)
{
  pcl::SACSegmentationFromNormals<T, pcl::Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iterations);
  seg.setDistanceThreshold(threshold);
  seg.setInputCloud(input);
  seg.setInputNormals(input_nor);
  seg.segment(*inliers_plane, *coefficients_plane);
}

template<typename T>
void
extractPlane(
    const typename pcl::PointCloud<T>::Ptr& input,
    const typename pcl::PointCloud<T>::Ptr& output,
    float thre)
{
  pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
  estimateNormal<T>(input, normal, 50);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
  segmentPlane<T>(input, normal, coefficients_plane, inliers_plane, thre);
  extractCloud<T>(input, inliers_plane, output, true);
}

template<typename T>
void
extractPlane(
    const typename pcl::PointCloud<T>::Ptr& input,
    const typename pcl::PointCloud<T>::Ptr& output,
    const typename pcl::PointCloud<T>::Ptr& plane,
    float thre)
{
  pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
  estimateNormal<T>(input, normal, 50);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
  segmentPlane<T>(input, normal, coefficients_plane, inliers_plane, thre);
  extractCloud<T>(input, inliers_plane, output, true);
  extractCloud<T>(input, inliers_plane, plane, false);
}

/*
  state: true:  remove points in the inliers
         false: keep points in the inliers (default)
*/
template<typename T>
void
extractCloud(
    const typename pcl::PointCloud<T>::Ptr& input,
    const typename pcl::PointIndices::Ptr inliers,
    const typename pcl::PointCloud<T>::Ptr& output,
    const bool state)
{
  pcl::ExtractIndices<T> extract;
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(state);
  extract.filter(*output);
}

template<typename T>
void
removeOutlier(
    const typename pcl::PointCloud<T>::Ptr& input,
    const typename pcl::PointCloud<T>::Ptr& output,
    const int meanK, const float thresh)
{
  pcl::StatisticalOutlierRemoval<T> sor;
  sor.setInputCloud(input);
  sor.setMeanK(meanK);
  sor.setStddevMulThresh(thresh);
  sor.filter(*output);
}

template<typename T>
int
extractClusters(
    const typename pcl::PointCloud<T>::Ptr& input,
    std::vector<typename pcl::PointCloud<T>::Ptr>& clusters,
    const float tolerance, const int max_size, const int min_size)
{
  if(input->size() == 0)
  {
    return 0;
  }
  clusters.clear();
  std::vector<pcl::PointIndices> cluster_indices;
  typename pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T>);
  tree->setInputCloud(input);
  pcl::EuclideanClusterExtraction<T> ec;
  ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input);
  ec.extract(cluster_indices);

  for (const auto& cluster : cluster_indices)
  {
    typename pcl::PointCloud<T>::Ptr cloud_cluster (new pcl::PointCloud<T>);
    for (const auto& idx : cluster.indices)
    {
      cloud_cluster->push_back((*input)[idx]);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    clusters.push_back(cloud_cluster);
  }
  return clusters.size();
}

template <typename T, typename pT>
int
obtainNearest(
    const typename pcl::KdTreeFLANN<T>::Ptr tree,
    const pT* spoint,
    float &dist)
{
  T point;
  pcl::copyPoint<pT, T> (*spoint, point);
  std::vector<int> idx(1);
  std::vector<float> sq_dist(1);
  tree->nearestKSearch(point, 1, idx, sq_dist);
  dist = sq_dist[0];
  return idx[0];
}

template<typename T>
bool
icpCloud(
    const typename pcl::PointCloud<T>::Ptr target,
    const typename pcl::PointCloud<T>::Ptr input,
    const int max_iter,
    const float max_dist,
    const float score_thre)
{
  pcl::IterativeClosestPoint<T, T> icp;
  icp.setInputSource(input);
  icp.setInputTarget(target);
  icp.setMaximumIterations(max_iter);
  icp.setMaxCorrespondenceDistance(max_dist);
  icp.align(*input);
  if (icp.getFitnessScore() < score_thre)
  {
    return true; // succeeded
  }
  return false;
}

template<typename T>
void
PCACloud(
    const typename pcl::PointCloud<T>::Ptr input,
    const typename pcl::PointCloud<T>::Ptr output)
{
  pcl::PCA<T> pca (new pcl::PCA<T>);
  pca.setInputCloud(input);
  pca.project(*input, *output);
}

template<typename T>
void
proj2XOZ(
    const typename pcl::PointCloud<T>::Ptr input,
    const xyzPtr output)
{
  pcl::PointXYZ tmp;
  tmp.y = 0;
  for (auto& point : input->points)
  {
    tmp.x = point.x;
    tmp.z = point.z;
    output->points.push_back(tmp);
  }
}

template <typename T>
void
widthXY(
    const typename pcl::PointCloud<T>::Ptr input,
    float& width_x,
    float& width_y)
{
  float x_min = 1;
  float x_max = -1;
  float y_min = 1;
  float y_max = -1;
  for (auto& point : input->points)
  {
    x_min = x_min < point.x ? x_min : point.x;
    x_max = x_max > point.x ? x_max : point.x;
    y_min = y_min < point.y ? y_min : point.y;
    y_max = y_max > point.y ? y_max : point.y;
  }
  width_x = x_max - x_min;
  width_y = y_max - y_min;
}

template <typename T>
float
minZ(const typename pcl::PointCloud<T>::Ptr input)
{
  float z_min = 1;
  for (auto& point : input->points)
  {
    z_min = z_min < point.z ? z_min : point.z;
  }
  return z_min;
}

template<typename T>
float
computeHeight(
    const typename pcl::PointCloud<T>::Ptr input)
{
  float sum = 0;
  for (auto& point : input->points)
  {
    sum += point.z;
  }
  return (sum / input->size());
}

template<typename T>
void
computeCentroid(
    const typename pcl::PointCloud<T>::Ptr input,
    Eigen::Vector3d &centroid)
{
  float x = 0, y = 0, z = 0;
  for (auto& point : input->points)
  {
    x += point.x;
    y += point.y;
    z += point.z;
  }
  centroid << x / input->size(), y / input->size(), z / input->size();
  return;
}

template<typename T>
void
computeMaxR(
  const typename pcl::PointCloud<T>::Ptr input,
  const Eigen::Vector3d &centroid,
  float &max_r)
{
  max_r = 0;
  float r;
  for (auto& point : input->points)
  {
    r = pow(point.x - centroid(0), 2) +
        pow(point.y - centroid(1), 2) +
        pow(point.z - centroid(2), 2);
    max_r = max_r > r ? max_r : r;
  }
  max_r = sqrt(max_r);
}

template<typename T>
bool
isExist(
    const typename pcl::PointCloud<T>::Ptr input,
    std::vector<float> min_max,
    const int num_threshold)
{
  typename pcl::PointCloud<T>::Ptr left (new pcl::PointCloud<T>);
  pass<T>(input, left, min_max[0], min_max[1], "x");
  pass<T>( left, left, min_max[2], min_max[3], "y");
  pass<T>( left, left, min_max[4], min_max[5], "z");
  if (left->size() > num_threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}