#include "ub_view.h"

#define PCV pcl::visualization::PCLVisualizer

struct timeval last_press[10];

void init_timers(){
  for(int idx = 0; idx < 10; idx++)
    gettimeofday(&last_press[idx], NULL);
}

// Need to smooth out flying. Key presses logged faster than can update scene.
bool process_press(int which_timer){
  struct timeval now;
  gettimeofday(&now, NULL);

  // convert to ms
  long mtime = (now.tv_sec - last_press[which_timer].tv_sec) * 1000 + (now.tv_usec - last_press[which_timer].tv_usec) / 1000.0;

  if(mtime > 1000.0 / PRESSPERSEC){
    gettimeofday(&last_press[which_timer], NULL);
    return true;
  }

  return false;
}

void keyboard_handler(const pcl::visualization::KeyboardEvent &event, void* pviewer){
  boost::shared_ptr<PCV> viewer = *static_cast<boost::shared_ptr<PCV> *> (pviewer);

  if (event.getKeyCode () == 0x0000005d && event.keyDown()){ //Right Bracket
    if( process_press(0)){
      camera_move('u', -1);

    }
  }

  if (event.getKeyCode () == 0x00000027 && event.keyDown()){ //Apostra
    if( process_press(1))
      camera_move('u', 1);
  }

  if (event.getKeyCode () == 0x0000005b && event.keyDown()){ //Left Bracket
    if( process_press(2))
      camera_move('v', 1);
  }

  if (event.getKeyCode () == 0x0000005c && event.keyDown()){ // backslash
    if( process_press(3))
      camera_move('v', -1);
  }

  if (event.getKeyCode() == 0x0000002e && event.keyDown()){ // Period Key
    if( process_press(4))
      camera_move('w', 1);
  }

  if (event.getKeyCode() == 0x0000002f && event.keyDown()){
    if( process_press(5))
      camera_move('w', -1);

  }

  if( event.getKeySym() == "z" && event.keyDown()){
    // face_origin()
  }
}
