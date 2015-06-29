#include "ub_view.h"

#define UVTITLE "UV Controls"

cv::Mat logo;

void gui_refresh(int a, void *p){
  cv::imshow(UVTITLE, logo);
}

void setup_highgui(){
  cv::namedWindow(UVTITLE, 0);

  logo = cv::imread("../uber.jpg");
  cv::createTrackbar("Red:       ", UVTITLE, &gui_states[0], 255, gui_refresh);
  cv::createTrackbar("Green:     ", UVTITLE, &gui_states[1], 255, gui_refresh);
  cv::createTrackbar("Blue:      ", UVTITLE, &gui_states[2], 255, gui_refresh);

  cv::createTrackbar("Heat Map:  ", UVTITLE, &gui_states[3], 1, gui_refresh);
  cv::createTrackbar("XYZ Axis:  ", UVTITLE, &gui_states[4], 3, gui_refresh);

  gui_refresh(0, NULL);

  cv::waitKey(0);
}
