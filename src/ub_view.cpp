// STDLIB
#include <stdio.h>
#include <stdlib.h>
#include <cfloat>

// Project
#include "ub_view.h"

int
main (int argc, char** argv)
{
  //pack_unpack_test();
  //binary_pcd_test();

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_mem;
  pcl::io::loadPCDFile ("../model.pcd", cloud_mem);

  cloud = (pcl::PointCloud<pcl::PointXYZ>::ConstPtr) &cloud_mem;


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Uber View"));
  viewer->setBackgroundColor( 0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "Model Cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Model Cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped ()){
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds ( 100000));
  }

  return EXIT_SUCCESS;
}
