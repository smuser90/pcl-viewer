#include "ub_view.h"

// Clear cloud
void clear_cloud(pcl::PointCloud<UberXYZ_RGB> *cloud){
  for(int pt=0; pt < cloud->size(); pt++)
    cloud->points[pt] = {0};// vertex normal
}

void print_point(UberXYZ_RGB* p){
  uint8_t rgb[3] = {0,0,0};
  unpack_rgb(p, rgb); // populate colors

  printf("Point %d: Coords{%f, %f, %f}\tR| %d\tG| %d\t B| %d\n", p->id,
    p->x, p->y, p->z,
    rgb[0], rgb[1], rgb[2]);
}

void print_cloud(pcl::PointCloud<UberXYZ_RGB> *cloud){
  for(int pt=0; pt < cloud->size(); pt++)
    print_point(&cloud->points[pt]);
}
