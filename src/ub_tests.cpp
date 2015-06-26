#include "ub_view.h"

#define PACKINGPOINTS 5

void pack_unpack_test(void){
  pcl::PointCloud<UberXYZ_RGB> cloud;
  cloud.points.resize (PACKINGPOINTS);
  cloud.width = PACKINGPOINTS;
  cloud.height = 1;

  //Initialize cloud
  for(int pt = 0; pt < cloud.size(); pt++){
    cloud.points[pt].id = pt+100;
    set_xyz(&cloud.points[pt], pt+1, pt+1, pt+1);
    pack_rgb(&cloud.points[pt] , colors[pt % ARRAY_LENGTH(colors)]);
  }

  // Save the file
  pcl::io::savePCDFile ("pack_unpack.pcd", cloud);

  clear_cloud(&cloud);
  // Load it back from disk
  pcl::io::loadPCDFile ("pack_unpack.pcd", cloud);

  for(int pt = 0; pt < cloud.size(); pt++){
    printf("RGB Before: %f\n", cloud.points[pt].rgb);
    print_point(&cloud.points[pt]);
  }
}

void binary_pcd_test(void){
  pcl::PointCloud<UberXYZ_RGB> cloud;

  // Load binary encoded non-rgb point cloud
  pcl::io::loadPCDFile ("../model.pcd", cloud);
  print_cloud(&cloud);

}
