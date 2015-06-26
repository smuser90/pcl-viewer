#include "ub_view.h"

void on_trackbar( int as, void* p){
  alpha = (double) alpha_slider / alpha_slider_max;
  beta = (1.0 - alpha);

  cv::addWeighted( src1, alpha, src2, beta, 0.0, dst );

  cv::imshow( "Ub-GUI", dst);
  int key = cv::waitKey(50);

  if(key == 27)
    exit(EXIT_SUCCESS);
}
