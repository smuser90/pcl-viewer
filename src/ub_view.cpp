#include "ub_view.h"

const int alpha_slider_max = 100;
int alpha_slider;
double alpha, beta;

cv::Mat src1, src2, dst;

void on_trackbar( int as, void* p){
  alpha = (double) alpha_slider / alpha_slider_max;
  beta = (1.0 - alpha);

  cv::addWeighted( src1, alpha, src2, beta, 0.0, dst );

  cv::imshow( "Ub-GUI", dst);
  int key = cv::waitKey(50);

  if(key == 27)
    exit(EXIT_SUCCESS);
}

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

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  tree->setInputCloud(cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch(20);
  n.compute (*normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *normal_cloud);

  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (normal_cloud);

  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  gp3.setSearchRadius(0.05);
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (200);
  gp3.setMaximumSurfaceAngle(M_PI / 4);
  gp3.setMinimumAngle(M_PI / 18);
  gp3.setMaximumAngle(2 * M_PI / 3);
  gp3.setNormalConsistency(false);

  gp3.setInputCloud (normal_cloud);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("Uber View"));
  viewer->setBackgroundColor( 0, 0, 0);

  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  //viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Model Cloud");
  viewer->addPolygonMesh( triangles, "Model");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Model");

  viewer->addCoordinateSystem(0.1);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

  cv::namedWindow("Ub-GUI", 1);

  char TrackbarName[50];
  sprintf( TrackbarName, "Alpha : %d", alpha_slider);
  cv::createTrackbar("BlendBar", "Ub-GUI", &alpha_slider, alpha_slider_max, &on_trackbar);

  while (!viewer->wasStopped ()){
    on_trackbar( alpha_slider, 0);
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds ( 100000));
  }

  return EXIT_SUCCESS;
}
