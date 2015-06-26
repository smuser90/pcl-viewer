#include "ub_view.h"

#define PCV pcl::visualization::PCLVisualizer

void keyboard_handler(const pcl::visualization::KeyboardEvent &event, void* pviewer){
  boost::shared_ptr<PCV> viewer = *static_cast<boost::shared_ptr<PCV> *> (pviewer);

  if (event.getKeySym () == "w" && event.keyDown()){

    viewer->getCameraParameters (ub_camera); // Keep camera fresh

    double *xyz = ub_camera.pos;
    double *lookat = ub_camera.focal;
    double res[3];

    for( int idx = 0; idx < 3; idx++)
      res[idx] = lookat[idx] - xyz[idx];

    float x = res[0];
    float y = res[1];
    float z = res[2];

    reduce_to_unit(x, y, z);

    ub_movement(0,0) = x * STEP;
    ub_movement(0,1) = y * STEP;
    ub_movement(0,2) = z * STEP;

    viewer->setCameraPosition(ub_movement(0,0) + ub_camera.pos[0],
        ub_movement(0,1) + ub_camera.pos[1],
        ub_movement(0,2) + ub_camera.pos[2],
        ub_camera.focal[0],  ub_camera.focal[1],   ub_camera.focal[2],
        ub_camera.view[0], ub_camera.view[1], ub_camera.view[2]);
  }

  if (event.getKeySym () == "s" && event.keyDown()){

    viewer->getCameraParameters (ub_camera); // Keep camera fresh

    double *xyz = ub_camera.pos;
    double *lookat = ub_camera.focal;
    double res[3];

    for( int idx = 0; idx < 3; idx++)
      res[idx] = xyz[idx] - lookat[idx];

    float x = res[0];
    float y = res[1];
    float z = res[2];

    reduce_to_unit(x, y, z);

    ub_movement(0,0) = x * STEP;
    ub_movement(0,1) = y * STEP;
    ub_movement(0,2) = z * STEP;

    viewer->setCameraPosition(ub_movement(0,0) + ub_camera.pos[0],
        ub_movement(0,1) + ub_camera.pos[1],
        ub_movement(0,2) + ub_camera.pos[2],
        ub_camera.focal[0],  ub_camera.focal[1],   ub_camera.focal[2],
        ub_camera.view[0], ub_camera.view[1], ub_camera.view[2]);
  }
}
