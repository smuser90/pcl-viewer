#include "ub_view.h"

#define PCV pcl::visualization::PCLVisualizer

void keyboard_handler(const pcl::visualization::KeyboardEvent &event, void* pviewer){
  boost::shared_ptr<PCV> viewer = *static_cast<boost::shared_ptr<PCV> *> (pviewer);

  if (event.getKeyCode () == 0x0000005d && event.keyDown()){ //Right Bracket
    camera_move('u', 1);
  }

  if (event.getKeyCode () == 0x00000027 && event.keyDown()){ //Apostra
    camera_move('u', -1);
  }

  if (event.getKeyCode () == 0x0000005b && event.keyDown()){ //Left Bracket
    camera_move('v', 1);
  }

  if (event.getKeyCode () == 0x0000005c && event.keyDown()){ // backslash
    camera_move('v', -1);
  }

  if (event.getKeyCode() == 0x0000002e && event.keyDown()){ // Period Key
    camera_move('w', 1);
  }

  if (event.getKeyCode() == 0x0000002f && event.keyDown()){
    camera_move('w', -1);

  }

  if( event.getKeySym() == "z" && event.keyDown()){
    if(!mesh_colored) color_mesh();
  }
}
