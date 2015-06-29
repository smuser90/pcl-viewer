#include "ub_view.h"

#define UVTITLE "UV Controls"

cv::Mat logo;

void gui_refresh(int a, void *p){
  cv::imshow(UVTITLE, logo);
}

void setup_highgui(){
  cv::namedWindow(UVTITLE, 0);

  logo = cv::imread("../uber.jpg");
  cv::createTrackbar("Red:       ", UVTITLE, &gui_states[GUIRED], 255, gui_refresh);
  cv::createTrackbar("Green:     ", UVTITLE, &gui_states[GUIGREEN], 255, gui_refresh);
  cv::createTrackbar("Blue:      ", UVTITLE, &gui_states[GUIBLUE], 255, gui_refresh);

  cv::createTrackbar("Heat Map:  ", UVTITLE, &gui_states[GUIHEATMAP], 1, gui_refresh);
  cv::createTrackbar("XYZ Axis:  ", UVTITLE, &gui_states[GUIXYZ], 2, gui_refresh);

  gui_refresh(0, NULL);

  cv::waitKey(0);
}
