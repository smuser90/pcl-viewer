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
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// BOOST
#include <boost/thread/thread.hpp>
#include <boost/timer/timer.hpp>

//VTK
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkLookupTable.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkProperty.h>


// STDLIB
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cfloat>
#include <ctime>
#include <cmath>
#include <iostream>

// Linux
#include <sys/time.h>

#define ARRAY_LENGTH(x) (sizeof(x)/sizeof(x[0]))
#define BYTE 8

#define STEP 0.005

#define PRESSPERSEC 8

#define GUIELEMENTS 10

#define GUIRED 0
#define GUIGREEN 1
#define GUIBLUE 2
#define GUIHEATMAP 3
#define GUIXYZ 4

// Globals
extern pcl::visualization::Camera ub_camera;
extern Eigen::Matrix4f ub_movement;

extern boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

extern vtkSmartPointer<vtkUnsignedCharArray> vtk_colors;
extern vtkSmartPointer<vtkLookupTable> vtk_colorLookupTable;

extern int gui_states[GUIELEMENTS];

extern struct timeval last_press[10];

void save_file();
void print_help(bool pcl);
void mesh_cloud();

// GUI
void setup_highgui();
void reset_view();

// Controls
void keyboard_handler(const pcl::visualization::KeyboardEvent &event, void* pviewer);
void init_timers(void);

// Camera
void camera_move(char vec, int dir);

// util
void clear_cloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud);
void print_cloud(pcl::PointCloud<pcl::PointXYZRGB> *cloud);
void print_point(pcl::PointXYZRGB *p);

void pack_rgb(pcl::PointXYZRGB *p, uint8_t rgb[3]);
void unpack_rgb(pcl::PointXYZRGB *p, uint8_t (&rgb)[3]);
void set_xyz(pcl::PointXYZRGB *p, float x, float y, float z);

void heat_map(pcl::PointCloud<pcl::PointXYZRGB> &cloud, int axis, vtkSmartPointer<vtkPolyData> vtk_mesh);
void heat_map_vtkmesh(vtkSmartPointer<vtkPolyData> raw_mesh, int axis);
void color_vtkmesh(vtkSmartPointer<vtkPolyData> raw_mesh, uint8_t rgb[3]);
void color_cloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, uint8_t rgb[3]);

void arb_rotate(Eigen::Matrix4f &transform, double theta_rad, Eigen::Vector3f &vec);

void estimate_normals(pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> &extract_normals,
  pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);

void stitch_mesh(pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud, pcl::PolygonMesh &triangles);

void cloud_to_mesh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PolygonMesh &mesh);

// tests
void pack_unpack_test(void);
void binary_pcd_test(void);

#endif
