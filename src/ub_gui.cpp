#include "ub_view.h"

cv::Mat logo;

void gui_refresh(int a, void *p){
  cv::imshow("Ub-GUI", logo);
}

void setup_highgui(){
  cv::namedWindow("Ub-GUI", 0);

  logo = cv::imread("../uber.jpg");
  cv::createTrackbar("Enable RGB:", "Ub-GUI", &enable_tbar, 1, gui_refresh);
  cv::createTrackbar("Red", "Ub-GUI", &red_tbar, 255, gui_refresh);
  cv::createTrackbar("Green", "Ub-GUI", &green_tbar, 255, gui_refresh);
  cv::createTrackbar("Blue", "Ub-GUI", &blue_tbar, 255, gui_refresh);

  gui_refresh(0, NULL);

  cv::waitKey(0);
}
