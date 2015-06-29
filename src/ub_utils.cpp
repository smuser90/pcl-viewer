#include "ub_view.h"


// Clear cloud
void clear_cloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud){
  for(int pt=0; pt < cloud->size(); pt++)
    set_xyz(&cloud->points[pt], 0, 0, 0);
}

void color_vtkmesh(vtkSmartPointer<vtkPolyData> raw_mesh /*, pcl::PointCloud<PointXYZRGB> parent |Color it from the parent cloud */){
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");

  double bounds[6];
  raw_mesh->GetBounds(bounds);

  vtkSmartPointer<vtkLookupTable> colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
  colorLookupTable->SetTableRange(bounds[2], bounds[3]);
  colorLookupTable->Build();

  for(int i = 0; i < raw_mesh->GetNumberOfPoints(); i++){
    double p[3];
    raw_mesh->GetPoint(i,p);

    double dcolor[3];
    colorLookupTable->GetColor(p[1], dcolor);

    unsigned char color[3];
    for(int j = 0; j < 3; j++)
      color[j] = static_cast<unsigned char>(255.0 * dcolor[j]);

    colors->InsertNextTupleValue(color);
  }

  raw_mesh->GetPointData()->SetScalars(colors);
}

// Colors points based on height
void heat_map(pcl::PointCloud<pcl::PointXYZRGB> &cloud, int axis){
  pcl::PointXYZRGB max, min;
  pcl::getMinMax3D( cloud, min, max);

  float range = max.y - min.y;
  uint8_t clr_max[3] = {255, 0, 0};
  uint8_t clr_min[3] = {0, 255, 0};
  uint8_t clr[3] = {0, 0, 0};

  for(int pt = 0; pt < cloud.size(); pt++){

    float percent_range;
    switch(axis){
      case 0:
        percent_range = ((max.x - cloud.points[pt].x) / range);
        break; // x

      case 1:
        percent_range = ((max.y - cloud.points[pt].y) / range);
        break; // y

      case 2:
        percent_range = ((max.z - cloud.points[pt].z) / range);
        break; // z
    }

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

void arb_rotate(Eigen::Matrix4f &transform, double theta_rad, Eigen::Vector3f &vec){
  float costheta = cos(theta_rad);
  float sintheta = sin(theta_rad);
  vec.normalize();

  transform = Eigen::Matrix4f::Identity();

  transform (0,0) = costheta + (vec(0)*vec(0)) * (1 - costheta);
  transform (0,1) = (vec(0) * vec(1)) * (1 - costheta) - vec(2) * sintheta;
  transform (0,2) = (vec(0) * vec(2)) * (1 - costheta) + vec(1) * sintheta;

  transform (1, 0) = (vec(1) * vec(0)) * (1 - costheta) + vec(2) * sintheta;
  transform (1, 1) = costheta + (vec(1) * vec(1)) * (1 - costheta);
  transform (1, 2) = (vec(1) * vec(2)) * (1 - costheta) + vec(0) * sintheta;

  transform (2, 0) = (vec(2) * vec(0)) * (1 - costheta) - vec(1) * sintheta;
  transform (2, 1) = (vec(2) * vec(1)) * (1 - costheta) + vec(0) * sintheta;
  transform (2, 2) = costheta + (vec(2) * vec(2)) * (1 - costheta);

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
