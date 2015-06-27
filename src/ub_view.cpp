#include "ub_view.h"


cv::Mat src1, src2, dst;
pcl::visualization::Camera ub_camera;
Eigen::Matrix4f ub_movement;


const int alpha_slider_max = 100;
int alpha_slider;
double alpha, beta;

pcl::PolygonMesh mesh;

bool mesh_colored = false;

vtkSmartPointer<vtkPolyData> raw_mesh = vtkSmartPointer<vtkPolyData>::New();

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

  cloud_to_mesh(cloud, mesh);

  pcl::VTKUtils::convertToVTK(mesh, raw_mesh);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Uber View"));
  viewer->setBackgroundColor( 0, 0, 0);

  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  //viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Model Cloud");
  //viewer->addPolygonMesh( mesh, "Model");
  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Model");


  viewer->addCoordinateSystem(0.1);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0.0, 0.0, 2.0, 0, 0.2, 0, 0.0, 1.0, 0.0);
  viewer->registerKeyboardCallback(keyboard_handler, (void *)&viewer);
  viewer->getCameraParameters (ub_camera);

  cv::namedWindow("Ub-GUI", 1);

  char TrackbarName[50];
  sprintf( TrackbarName, "Alpha : %d", alpha_slider);
  cv::createTrackbar("BlendBar", "Ub-GUI", &alpha_slider, alpha_slider_max, &on_trackbar);
  on_trackbar( alpha_slider, 0);

  while (!viewer->wasStopped ()){

    viewer->spinOnce(100);

    boost::this_thread::sleep (boost::posix_time::microseconds ( 10000));
  }

  return EXIT_SUCCESS;
}
