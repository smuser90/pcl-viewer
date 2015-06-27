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

  if (event.getKeySym () == "a" && event.keyDown()){
    viewer->getCameraParameters (ub_camera);

    Eigen::Vector3f look;
    Eigen::Vector3f up;
    for(int idx = 0; idx < 3; idx++){
      look(idx) = ub_camera.pos[idx] - ub_camera.focal[idx];
      up(idx) = ub_camera.view[idx];
    }
    look.normalize();
    up.normalize();

    Eigen::Vector3f right = look.cross(up);
    right.normalize();

    ub_movement(0,0) = right(0) * STEP;
    ub_movement(0,1) = right(1) * STEP;
    ub_movement(0,2) = right(2) * STEP;

    viewer->setCameraPosition(ub_movement(0,0) + ub_camera.pos[0],
        ub_movement(0,1) + ub_camera.pos[1],
        ub_movement(0,2) + ub_camera.pos[2],
        ub_camera.focal[0] + ub_movement(0,0),
        ub_movement(0,1) + ub_camera.focal[1],
        ub_movement(0,2) + ub_camera.focal[2],
        ub_camera.view[0], ub_camera.view[1], ub_camera.view[2]);
  }

  if (event.getKeySym () == "d" && event.keyDown()){
    viewer->getCameraParameters (ub_camera);

    Eigen::Vector3f look;
    Eigen::Vector3f up;
    for(int idx = 0; idx < 3; idx++){
      look(idx) = ub_camera.pos[idx] - ub_camera.focal[idx];
      up(idx) = ub_camera.view[idx];
    }
    look.normalize();
    up.normalize();

    Eigen::Vector3f right = look.cross(up);
    right.normalize();

    ub_movement(0,0) = -1 * right(0) * STEP;
    ub_movement(0,1) = -1 * right(1) * STEP;
    ub_movement(0,2) = -1 * right(2) * STEP;

    viewer->setCameraPosition(ub_movement(0,0) + ub_camera.pos[0],
        ub_movement(0,1) + ub_camera.pos[1],
        ub_movement(0,2) + ub_camera.pos[2],
        ub_camera.focal[0] + ub_movement(0,0),
        ub_movement(0,1) + ub_camera.focal[1],
        ub_movement(0,2) + ub_camera.focal[2],
        ub_camera.view[0], ub_camera.view[1], ub_camera.view[2]);
  }

  if (event.getKeySym() == "z" && event.keyDown()){
    viewer->getCameraParameters (ub_camera);

    viewer->setCameraPosition( ub_camera.pos[0], ub_camera.pos[1] + STEP, ub_camera.pos[2],
      ub_camera.focal[0], ub_camera.focal[1], ub_camera.focal[2],
      ub_camera.view[0], ub_camera.view[1], ub_camera.view[2]);
  }

  if ( event.isCtrlPressed() && event.keyDown()){
    viewer->getCameraParameters (ub_camera);
    viewer->setCameraPosition( ub_camera.pos[0], ub_camera.pos[1] - STEP, ub_camera.pos[2], 
      ub_camera.focal[0], ub_camera.focal[1], ub_camera.focal[2],
      ub_camera.view[0], ub_camera.view[1], ub_camera.view[2]);

  }


}
