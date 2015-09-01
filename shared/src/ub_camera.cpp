#include "ub_view.h"

struct move_flag {
  unsigned char flags[9];
};

void calc_resultant(double *v1, double *v2, Eigen::Vector3f &res){
    for( int idx=0; idx < 3; idx++)
      res(idx) = v1[idx] - v2[idx];

    res.normalize();
}

void init_move(Eigen::Vector3f vec, int dir){
  for(int idx=0; idx<3; idx++)
    ub_movement(0,idx) = vec(idx) * STEP * dir;
}

void update_move(unsigned char flags[9]){
  viewer->setCameraPosition(
    ub_movement(0,0) * flags[0] + ub_camera.pos[0],
    ub_movement(0,1) * flags[1] + ub_camera.pos[1],
    ub_movement(0,2) * flags[2] + ub_camera.pos[2],
    ub_movement(0,0) * flags[3] + ub_camera.focal[0],
    ub_movement(0,1) * flags[4] + ub_camera.focal[1],
    ub_movement(0,2) * flags[5] + ub_camera.focal[2],
    ub_movement(0,0) * flags[6] + ub_camera.view[0],
    ub_movement(0,1) * flags[7] + ub_camera.view[1],
    ub_movement(0,2) * flags[8] + ub_camera.view[2]
  );
}

// camera_move(char vec, int dir)
void camera_move(char vec, int dir){
  dir = dir / abs(dir); // Make sure this is +/-1

  viewer->getCameraParameters (ub_camera); // Get fresh camera state

  double *from;
  double *to;

  Eigen::Vector3f res;
  Eigen::Vector3f up;

  struct move_flag update_flags;

  switch(vec){
    case 'u':
      from = ub_camera.pos;
      to = ub_camera.focal;

      calc_resultant(from, to, res);

      init_move(res, dir);

      update_flags = {1,1,1,0,0,0,0,0,0};
      update_move(update_flags.flags);
      break;

    case 'v':
      from = ub_camera.pos;
      to = ub_camera.focal;

      calc_resultant(from, to, res);

      up(0) = ub_camera.view[0];
      up(1) = ub_camera.view[1];
      up(2) = ub_camera.view[2];

      res = res.cross(up);
      init_move(res, dir);

      update_flags = {1,1,1,1,1,1,0,0,0};
      update_move(update_flags.flags);
      break;

    case 'w':
      from = ub_camera.pos;
      to = ub_camera.view;

      calc_resultant(from, to, res);

      init_move(res, dir);

      update_flags = {1,1,1,1,1,1,0,0,0};
      update_move(update_flags.flags);
      break;
  }
}
