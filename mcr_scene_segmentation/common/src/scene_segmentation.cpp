#include <mcr_scene_segmentation/scene_segmentation.h>

SceneSegmentation::SceneSegmentation()
{
    cluster_extraction.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT> >());
    normal_estimation.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT> >());
}

SceneSegmentation::~SceneSegmentation()
{
}

//pointers which point to new point cloud objects
PointCloud::Ptr filtered(new PointCloud);
PointCloud::Ptr plane(new PointCloud);
PointCloud::Ptr hull(new PointCloud);
pcl::PointIndices::Ptr segmented_cloud_inliers(new pcl::PointIndices);
PointCloudN::Ptr normals(new PointCloudN);
PointCloudN::Ptr circle_normals(new PointCloudN);

std::vector<pcl::PointIndices> clusters_indices;

pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

pcl::ModelCoefficients::Ptr circle_coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr circle_inliers(new pcl::PointIndices);

PointCloud::Ptr SceneSegmentation::getPlane(const PointCloud::ConstPtr &cloud)
{


  //voxel_grid downsamples a PCL using 3D boxes
  voxel_grid.setInputCloud(cloud);
  voxel_grid.filter(*filtered);

  //pass_through is a crop of the PCL aling desired axes
  pass_through.setInputCloud(filtered);
  pass_through.filter(*filtered);

  //directly compute the surface normals at each point of the cloud
  normal_estimation.setInputCloud(filtered);
  normal_estimation.compute(*normals);

  //initializing the SAC model to detect planes
  sac.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
  sac.setMethodType(pcl::SAC_RANSAC);
  //to detect the points which are perpendicular to the place (z-axis)
  sac.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));

  sac.setInputCloud(filtered);
  sac.setInputNormals(normals);
  sac.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
      return hull;
  }

  //projecting the points onto a parametric model given by the coefficients
  project_inliers.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
  project_inliers.setInputCloud(filtered);
  project_inliers.setModelCoefficients(coefficients);
  project_inliers.setIndices(inliers);
  project_inliers.setCopyAllData(false);
  project_inliers.filter(*plane);

  //hull acts like a boundary of the detected plane
  convex_hull.setInputCloud(plane);
  convex_hull.reconstruct(*hull);

  // not sure if this is necessary
  hull->points.push_back(hull->points.front());
  hull->width += 1;
  return hull;
}

PointCloud::Ptr SceneSegmentation::getBoxes(const PointCloud::ConstPtr &cloud, PointCloud::ConstPtr &plane_cloud, std::vector<PointCloud::Ptr> &clusters, std::vector<BoundingBox> &boxes)
{

  pcl::PointIndices::Ptr segmented_cloud_inliers(new pcl::PointIndices);

  std::vector<pcl::PointIndices> clusters_indices;

  extract_polygonal_prism.setInputPlanarHull(hull);
  extract_polygonal_prism.setInputCloud(cloud);
  extract_polygonal_prism.setViewPoint(0.0, 0.0, 2.0);
  extract_polygonal_prism.segment(*segmented_cloud_inliers);

  cluster_extraction.setInputCloud(cloud);
  cluster_extraction.setIndices(segmented_cloud_inliers);

  cluster_extraction.extract(clusters_indices);

  const Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  for (size_t i = 0; i < clusters_indices.size(); i++)
  {
      const pcl::PointIndices& cluster_indices = clusters_indices[i];
      PointCloud::Ptr cluster(new PointCloud);
      pcl::copyPointCloud(*cloud, cluster_indices, *cluster);
      clusters.push_back(cluster);
      BoundingBox box = BoundingBox::create(cluster->points, normal);
      boxes.push_back(box);
  }
  return filtered;
}

void SceneSegmentation::getCircleParams(const PointCloud::Ptr circle_cloud, double &radius, double &center_x, double &center_y)
{

  {
    ROS_INFO("------------ Initiating the circle parameters------------------");
    //initializing the SAC model to detect circle
    seg.setModelType(pcl::SACMODEL_CIRCLE2D);
    ROS_INFO("------------ Setting the model------------------");
    seg.setMethodType(pcl::SAC_RANSAC);
    ROS_INFO("------------ Set the method------------------");

    seg.setInputCloud(circle_cloud);
    ROS_INFO("------------ Set input cloud ------------------");
    seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));

    seg.segment(*circle_inliers,*circle_coefficients);
    ROS_INFO("------------ Segmented circle------------------");
    center_x = circle_coefficients->values[0];
    center_y = circle_coefficients->values[1];
    radius = circle_coefficients->values[2];
  }
}


void SceneSegmentation::setVoxelGridParams(double leaf_size, const std::string &filter_field, double limit_min, double limit_max)
{
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.setFilterFieldName(filter_field);
    voxel_grid.setFilterLimits(limit_min, limit_max);
}

void SceneSegmentation::setPassthroughParams(const std::string &field_name, double limit_min, double limit_max)
{
    pass_through.setFilterFieldName(field_name);
    pass_through.setFilterLimits(limit_min, limit_max);
}

void SceneSegmentation::setNormalParams(double radius_search)
{
    normal_estimation.setRadiusSearch(radius_search);
}
void SceneSegmentation::setSACParams(int max_iterations, double distance_threshold, bool optimize_coefficients, double eps_angle, double normal_distance_weight)
{
    sac.setMaxIterations(max_iterations);
    sac.setDistanceThreshold(distance_threshold);
    sac.setEpsAngle(eps_angle);
    sac.setOptimizeCoefficients(optimize_coefficients);
    sac.setNormalDistanceWeight(normal_distance_weight);
}
void SceneSegmentation::setPrismParams(double min_height, double max_height)
{
    extract_polygonal_prism.setHeightLimits(min_height, max_height);
}

void SceneSegmentation::setOutlierParams(double radius_search, int min_neighbors)
{
    radius_outlier.setRadiusSearch(radius_search);
    radius_outlier.setMinNeighborsInRadius(min_neighbors);
}
void SceneSegmentation::setClusterParams(double cluster_tolerance, int cluster_min_size, int cluster_max_size, double cluster_min_height, double cluster_max_height, double max_length, double cluster_min_distance_to_polygon)
{
    cluster_extraction.setClusterTolerance(cluster_tolerance);
    cluster_extraction.setMinClusterSize(cluster_min_size);
    cluster_extraction.setMaxClusterSize(cluster_max_size);
}
