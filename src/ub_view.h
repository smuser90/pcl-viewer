#ifndef UBVIEW_H
#define UBVIEW_H

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>

// BOOST
#include <boost/thread/thread.hpp>

// STDLIB
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cfloat>
#include <iostream>

#define ARRAY_LENGTH(x) (sizeof(x)/sizeof(x[0]))

/* Following Boilerplate largely copy pasta'd from pointclouds.org */
struct UberXYZ_RGB
{
  uint32_t id;
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float rgb;
  float nx;                         // normal unit vector
  float ny;
  float nz;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (UberXYZ_RGB,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
)

//UberXYZ_RGB Dot Data
void pack_rgb(pcl::PointXYZRGB *p, uint8_t rgb[3]);
void unpack_rgb(pcl::PointXYZRGB *p, uint8_t (&rgb)[3]);
void set_xyz(pcl::PointXYZRGB *p, float x, float y, float z);

//util
void clear_cloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud);
void print_cloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud);
void print_point(pcl::PointXYZRGB *p);

void reduce_to_unit(float &x, float &y, float &z);
void arb_rotate(Eigen::Matrix4f &transform, double theta_rad, float x, float y, float z);

//tests
void pack_unpack_test(void);
void binary_pcd_test(void);

//globals
static uint8_t colors[5][3] = {
  {255, 128, 0},
  {0, 255, 128},
  {128, 0, 255},
  {100, 100, 100},
  {200, 200, 200}
};

#endif
