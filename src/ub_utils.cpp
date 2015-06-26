#include "ub_view.h"

// Clear cloud
void clear_cloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud){
  for(int pt=0; pt < cloud->size(); pt++)
    set_xyz(&cloud->points[pt], 0, 0, 0);
}

// Colors points based on height
void heat_map(pcl::PointCloud<pcl::PointXYZRGB> &cloud){
  pcl::PointXYZRGB max, min;
  pcl::getMinMax3D( cloud, min, max);

  float range = max.y - min.y;
  uint8_t clr_max[3] = {255, 0, 0};
  uint8_t clr_min[3] = {0, 255, 0};
  uint8_t clr[3] = {0, 0, 0};

  for(int pt = 0; pt < cloud.size(); pt++){

    float percent_range = ((max.y - cloud.points[pt].y) / range);
    clr[0] = clr_max[0] * percent_range;
    clr[1] = clr_min[1] * (1 - percent_range);

    pack_rgb( &cloud.points[pt], clr);
  }
}

// Pack bytes into float (allegedly helps with SSE performance)
void pack_rgb(pcl::PointXYZRGB *p, uint8_t rgb[3]){
	uint32_t rgb_mask = ((uint32_t)rgb[0] << 16 | (uint32_t)rgb[1] << 8 | (uint32_t)rgb[2]);
	p->rgb = *reinterpret_cast<float*>(&rgb_mask); // Type coursion int -> float
}

// Unpack routine
void unpack_rgb(pcl::PointXYZRGB *p, uint8_t (&rgb)[3]){
  uint32_t rgb_mask = *reinterpret_cast<uint32_t*>(&p->rgb);
  rgb[0] = (uint8_t)(rgb_mask >> 2 * BYTE);
  rgb[1] = (uint8_t)(rgb_mask >> BYTE);
  rgb[2] = (uint8_t)(rgb_mask);
}

// Set a points coords
void set_xyz(pcl::PointXYZRGB *p, float x, float y, float z){
  p->x = x;
  p->y = y;
  p->z = z;
}

void print_point(pcl::PointXYZRGB *p){
  uint8_t rgb[3] = {0,0,0};
  unpack_rgb(p, rgb); // populate colors

  printf("Coords{%f, %f, %f}\tR| %d\tG| %d\t B| %d\n",
    p->x, p->y, p->z,
    rgb[0], rgb[1], rgb[2]);
}

void print_cloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud){
  for(int pt=0; pt < cloud->size(); pt++)
    print_point(&cloud->points[pt]);
}

void reduce_to_unit(float &x, float &y, float &z){
  double l = sqrt(x*x + y*y  + z*z);
  x /= l;
  y /= l;
  z /= l;
}

void arb_rotate(Eigen::Matrix4f &transform, double theta_rad, float x, float y, float z){
  float costheta = cos(theta_rad);
  float sintheta = sin(theta_rad);
  reduce_to_unit(x, y, z);

  transform = Eigen::Matrix4f::Identity();

  transform (0,0) = costheta + (x*x) * (1 - costheta);
  transform (0,1) = (x * y) * (1 - costheta) - z * sintheta;
  transform (0,2) = (x * z) * (1 - costheta) + y * sintheta;

  transform (1, 0) = (y * x) * (1 - costheta) + z * sintheta;
  transform (1, 1) = costheta + (y * y) * (1 - costheta);
  transform (1, 2) = (y * z) * (1 - costheta) + x * sintheta;

  transform (2, 0) = (z * x) * (1 - costheta) - y * sintheta;
  transform (2, 1) = (z * y) * (1 - costheta) + x * sintheta;
  transform (2, 2) = costheta + (z * z) * (1 - costheta);

  transform (3, 3) = 1.0;
}

void estimate_normals(pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> &extract_normals,
  pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree){

  extract_normals.setInputCloud(tree->getInputCloud());
  extract_normals.setSearchMethod(tree);
  extract_normals.setKSearch(20);
  extract_normals.compute(*normals);
}

void stitch_mesh(pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud, pcl::PolygonMesh &triangles){
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(normal_cloud);

  pcl::GreedyProjectionTriangulation<pcl::PointNormal> stitch;

  stitch.setSearchRadius(0.05);
  stitch.setMu (2.5);
  stitch.setMaximumNearestNeighbors (200);
  stitch.setMaximumSurfaceAngle(M_PI / 4);
  stitch.setMinimumAngle(M_PI / 18);
  stitch.setMaximumAngle(2 * M_PI / 3);
  stitch.setNormalConsistency(false);

  stitch.setInputCloud (normal_cloud);
  stitch.setSearchMethod (tree2);
  stitch.reconstruct (triangles);
}
void cloud_to_mesh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PolygonMesh &mesh){

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> extract_normals;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  tree->setInputCloud(cloud);
  estimate_normals(extract_normals, normals, tree);

  pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud, *normals, *normal_cloud);

  stitch_mesh(normal_cloud, mesh);
}
