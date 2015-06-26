#ifndef UBVIEW_H
#define UBVIEW_H

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/eigen.h>
#include <pcl/visualization/vtk.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/histogram_visualizer.h>

#if VTK_MAJOR_VERSION==6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
  #include <pcl/visualization/pcl_plotter.h>
#endif

#include <pcl/visualization/point_picking_event.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>

/* Following Boilerplate largely copy pasta'd from pointclouds.org */
struct UberXYZ_RGB
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float rgb;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (UberXYZ_RGB,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
)

void pack_rgb(UberXYZ_RGB *p, uint8_t rgb[3]);
void unpack_rgb(UberXYZ_RGB *p, uint8_t (&rgb)[3]);

#endif
