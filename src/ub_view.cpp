#include "ub_view.h"

int
main (int argc, char** argv)
{
  //pack_unpack_test();
  //binary_pcd_test();

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_mem;

  pcl::io::loadPCDFile ("../model.pcd", cloud_mem);
  uint8_t clr[3] = {255, 0, 0};
  for(int pt = 0; pt < cloud_mem.size(); pt++)
  {
    pack_rgb( &cloud_mem.points[pt], clr);
  }

  cloud = (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr) &cloud_mem;
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Uber View"));
  viewer->setBackgroundColor( 0, 0, 0);

  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Model Cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Model Cloud");
  viewer->addCoordinateSystem(0.1);
  viewer->initCameraParameters();

  while (!viewer->wasStopped ()){
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds ( 100000));
  }

  return EXIT_SUCCESS;
}
