#include "ub_view.h"

#define PACKINGPOINTS 5

static uint8_t colors[5][3] = {
  {255, 128, 0},
  {0, 255, 128},
  {128, 0, 255},
  {100, 100, 100},
  {200, 200, 200}
};

// Getting rgb data out of float requires some shift-masking
// Make sure we did that right
void pack_unpack_test(void){

  // Initialize cloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.points.resize (PACKINGPOINTS);
  cloud.width = PACKINGPOINTS;
  cloud.height = 1;

  for(int pt = 0; pt < cloud.size(); pt++){
    set_xyz(&cloud.points[pt], pt+1, pt+1, pt+1);
    pack_rgb(&cloud.points[pt] , colors[pt % ARRAY_LENGTH(colors)]);
  }

  // Save the file
  pcl::io::savePCDFile ("pack_unpack.pcd", cloud);

  // Data? What data
  clear_cloud(&cloud);

  // Load it back from disk
  pcl::io::loadPCDFile ("pack_unpack.pcd", cloud);

  for(int pt = 0; pt < cloud.size(); pt++){
    print_point(&cloud.points[pt]);
  }
}

// Make sure we can load colorless files into rgb clouds.
void binary_pcd_test(void){
  pcl::PointCloud<pcl::PointXYZRGB> cloud;

  // Load binary encoded non-rgb point cloud
  pcl::io::loadPCDFile ("../model.pcd", cloud);
  print_cloud(&cloud);
}
