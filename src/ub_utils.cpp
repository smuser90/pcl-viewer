#include "ub_view.h"

// Clear cloud
void clear_cloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud){
  for(int pt=0; pt < cloud->size(); pt++)
    set_xyz(&cloud->points[pt], 0, 0, 0);
}

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
