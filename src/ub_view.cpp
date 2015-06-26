#include "ub_view.h"


cv::Mat src1, src2, dst;
pcl::visualization::Camera ub_camera;
Eigen::Matrix4f ub_movement;
const int alpha_slider_max = 100;
int alpha_slider;
double alpha, beta;

/*
void update_camera(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer){
  viewer->setCameraPosition(ub_movement(0,0) + ub_camera.pos[0],
      ub_movement(0,1) + ub_camera.pos[1],
      ub_movement(0,2) + ub_camera.pos[2],
      ub_camera.focal[0],  ub_camera.focal[1],   ub_camera.focal[2],
      ub_camera.view[0], ub_camera.view[1], ub_camera.view[2]);

  viewer->getCameraParameters (ub_camera); // Keep camera fresh

  for(int idx = 0; idx < 3; idx++)
    ub_movement(0, idx) = 0;
}
*/

int
main (int argc, char** argv)
{
  //pack_unpack_test();
  //binary_pcd_test();

  src1 = cv::imread("../viz.jpg");
  src2 = cv::imread("../rocket.jpg");

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;   // Most functions seem to want a const ptr
  pcl::PointCloud<pcl::PointXYZ> cloud_mem;         // Allocate on stack for now.

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_rgb;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_rgbmem;

  pcl::io::loadPCDFile ("../model.pcd", cloud_mem);
  cloud = (pcl::PointCloud<pcl::PointXYZ>::ConstPtr) &cloud_mem;

  pcl::PolygonMesh mesh;
  cloud_to_mesh(cloud, mesh);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Uber View"));
  viewer->setBackgroundColor( 0, 0, 0);

  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  //viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Model Cloud");
  viewer->addPolygonMesh( mesh, "Model");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Model");

  viewer->addCoordinateSystem(0.1);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  viewer->registerKeyboardCallback(keyboard_handler, (void *)&viewer);
  viewer->getCameraParameters (ub_camera);

  cv::namedWindow("Ub-GUI", 1);

  char TrackbarName[50];
  sprintf( TrackbarName, "Alpha : %d", alpha_slider);
  cv::createTrackbar("BlendBar", "Ub-GUI", &alpha_slider, alpha_slider_max, &on_trackbar);

  while (!viewer->wasStopped ()){
    //on_trackbar( alpha_slider, 0);

    //update_camera(viewer);

    viewer->spinOnce(100);

    boost::this_thread::sleep (boost::posix_time::microseconds ( 10000));
  }

  return EXIT_SUCCESS;
}
