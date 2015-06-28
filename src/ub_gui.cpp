#include "ub_view.h"

cv::Mat logo;

void gui_refresh(int a, void *p){
  cv::imshow("Ub-GUI", logo);
}

void setup_highgui(){
  cv::namedWindow("Ub-GUI", 0);

  logo = cv::imread("../uber.jpg");
  cv::createTrackbar("Enable RGB:", "Ub-GUI", &gui_states[0], 1, gui_refresh);
  cv::createTrackbar("Red:       ", "Ub-GUI", &gui_states[1], 255, gui_refresh);
  cv::createTrackbar("Green:     ", "Ub-GUI", &gui_states[2], 255, gui_refresh);
  cv::createTrackbar("Blue:      ", "Ub-GUI", &gui_states[3], 255, gui_refresh);

  cv::createTrackbar("Heat Map:  ", "Ub-GUI", &gui_states[4], 1, gui_refresh);
  cv::createTrackbar("XYZ Axis:  ", "Ub-GUI", &gui_states[5], 3, gui_refresh);

  gui_refresh(0, NULL);

  cv::waitKey(0);
}
