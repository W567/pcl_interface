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

template<typename T>
void
pass(
    const typename pcl::PointCloud<T>::Ptr &input,
    const typename pcl::PointCloud<T>::Ptr &output,
    const float min, const float max, const std::string axis, const bool state)
{
  pcl::PassThrough<T> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName (axis);
  pass.setFilterLimits (min, max);
  pass.setFilterLimitsNegative (state);
  pass.filter (*output);
}

template<typename T>
void
estimateNormal(
    const typename pcl::PointCloud<T>::Ptr& input,
    const pcl::PointCloud<pcl::Normal>::Ptr& output_nor,
    const int rangeK)
{
  pcl::NormalEstimation<T, pcl::Normal> ne;
  typename pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T>);
  ne.setInputCloud (input);
  ne.setSearchMethod (tree);
  ne.setKSearch (rangeK);
  ne.compute (*output_nor);
}

template<typename T>
void
segmentPlane(
    const typename pcl::PointCloud<T>::Ptr& input,
    const typename pcl::PointCloud<pcl::Normal>::Ptr& input_nor,
    const pcl::ModelCoefficients::Ptr& coefficients_plane,
    const pcl::PointIndices::Ptr& inliers_plane,
    const float threshold)
{
  pcl::SACSegmentationFromNormals<T, pcl::Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (2000);
  seg.setDistanceThreshold (threshold);
  seg.setInputCloud (input);
  seg.setInputNormals (input_nor);
  seg.segment (*inliers_plane, *coefficients_plane);
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

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end();
       ++it)
  {
    typename pcl::PointCloud<T>::Ptr cloud_cluster (new pcl::PointCloud<T>);
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end();
         ++pit)
    {
      cloud_cluster->points.push_back(input->points[*pit]);
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
    }
    clusters.push_back(cloud_cluster);
    j++;
  }
  return j;
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
    float* centroid)
{
  float x_sum = 0;
  float y_sum = 0;
  float z_sum = 0;
  for (auto& point : input->points)
  {
    x_sum += point.x;
    y_sum += point.y;
    z_sum += point.z;
  }
  centroid[0] = x_sum / input->size();
  centroid[1] = y_sum / input->size();
  centroid[2] = z_sum / input->size();
  return;
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
bool
icpCloud(
    const typename pcl::PointCloud<T>::Ptr target,
    const typename pcl::PointCloud<T>::Ptr input)
{
  pcl::IterativeClosestPoint<T, T> icp;
  icp.setInputSource(input);
  icp.setInputTarget(target);
  icp.setMaximumIterations(50);
  icp.setMaxCorrespondenceDistance(0.01f);
  icp.align(*input);
  if (icp.getFitnessScore() < 0.0001)
  {
    return true; // succeeded to align
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
    
    // y_min = point.y > 0 ? (point.y > y_min ? y_min : point.y) : y_min;
    // y_max = point.y < 0 ? (point.y < y_max ? y_max : point.y) : y_max;
    y_min = y_min < point.y ? y_min : point.y;
    y_max = y_max > point.y ? y_max : point.y;
  }

  width_x = x_max - x_min;
  width_y = y_max - y_min;
}
