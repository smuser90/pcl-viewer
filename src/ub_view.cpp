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

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_mem;

  pcl::io::loadPCDFile ("../model.pcd", cloud_mem);
  uint8_t clr[3] = {0, 240, 0};
  for(int pt = 0; pt < cloud_mem.size(); pt++){
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
  viewer->setCameraPosition(0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

  printf("Creating window");
  cv::namedWindow("Ub-GUI", 1);

  char TrackbarName[50];
  sprintf( TrackbarName, "Alpha : %d", alpha_slider);
  cv::createTrackbar("Hello Buttons", "Ub-GUI", &alpha_slider, alpha_slider_max, &on_trackbar);



  while (!viewer->wasStopped ()){
    on_trackbar( alpha_slider, 0);
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds ( 100000));
  }

  return EXIT_SUCCESS;
}
