#ifndef UBVIEW_H
#define UBVIEW_H

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkPolyDataReader.h>

#include <iostream>
#include <boost/thread/thread.hpp>

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
void pack_rgb(UberXYZ_RGB *p, uint8_t rgb[3]);
void unpack_rgb(UberXYZ_RGB *p, uint8_t (&rgb)[3]);
void set_xyz(UberXYZ_RGB *p, float x, float y, float z);
void set_nxyz(UberXYZ_RGB *p, float x, float y, float z);

//util
void clear_cloud(pcl::PointCloud<UberXYZ_RGB> *cloud);
void print_cloud(pcl::PointCloud<UberXYZ_RGB> *cloud);
void print_point(UberXYZ_RGB *p);

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
