#ifndef UBVIEW_H
#define UBVIEW_H

// Point Cloud Library
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// BOOST
#include <boost/thread/thread.hpp>

// STDLIB
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cfloat>
#include <iostream>

#define ARRAY_LENGTH(x) (sizeof(x)/sizeof(x[0]))
#define BYTE 8

// Globals (god forgive me)
extern Eigen::Matrix4f ub_camera;
extern const int alpha_slider_max;
extern int alpha_slider;
extern double alpha, beta;
extern cv::Mat src1, src2, dst;

// GUI
void on_trackbar( int as, void* p);

// util
void clear_cloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud);
void print_cloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud);
void print_point(pcl::PointXYZRGB *p);

void pack_rgb(pcl::PointXYZRGB *p, uint8_t rgb[3]);
void unpack_rgb(pcl::PointXYZRGB *p, uint8_t (&rgb)[3]);
void set_xyz(pcl::PointXYZRGB *p, float x, float y, float z);

void heat_map(pcl::PointCloud<pcl::PointXYZRGB> &cloud);

void reduce_to_unit(float &x, float &y, float &z);
void arb_rotate(Eigen::Matrix4f &transform, double theta_rad, float x, float y, float z);

// tests
void pack_unpack_test(void);
void binary_pcd_test(void);

// globals
static uint8_t colors[5][3] = {
  {255, 128, 0},
  {0, 255, 128},
  {128, 0, 255},
  {100, 100, 100},
  {200, 200, 200}
};

#endif
