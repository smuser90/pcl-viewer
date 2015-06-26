#include "ub_view.h"

#define BYTE 8

// Pack bytes into float (allegedly helps with SSE performance)
void pack_rgb(pcl::PointXYZRGB *p, uint8_t rgb[3]){
	uint32_t rgb_mask = ((uint32_t)rgb[0] << 16 | (uint32_t)rgb[1] << 8 | (uint32_t)rgb[2]);
	p->rgb = *reinterpret_cast<float*>(&rgb_mask); // Type coursion int -> float
}

// Unpack routine
void unpack_rgb(pcl::PointXYZRGB *p, uint8_t (&rgb)[3]){
  //printf("RGB Before: %f\n", p->rgb);
  uint32_t rgb_mask = *reinterpret_cast<uint32_t*>(&p->rgb);
  rgb[0] = (uint8_t)(rgb_mask >> 2 * BYTE);
  rgb[1] = (uint8_t)(rgb_mask >> BYTE);
  rgb[2] = (uint8_t)(rgb_mask);
  //printf("RGB After: %f\n", p->rgb);
}

void set_xyz(pcl::PointXYZRGB *p, float x, float y, float z){
  p->x = x;
  p->y = y;
  p->z = z;
}
