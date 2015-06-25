#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#define BYTE 8
/* Following Boilerplate largely copy pasta'd from pointclouds.org */
struct UberXYZ_RGB
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float rgb;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (UberXYZ_RGB,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
)

// Pack bytes into float (allegedly helps with SSE performance)
void pack_rgb(UberXYZ_RGB *p, uint8_t rgb[3]){
	uint32_t rgb_mask = ((uint32_t)rgb[0] << 16 | (uint32_t)rgb[1] << 8 | (uint32_t)rgb[2]);
	p->rgb = *reinterpret_cast<float*>(&rgb_mask); // Type coursion int -> float
}

// Unpack routine TODO: break these into utils file
void unpack_rgb(UberXYZ_RGB *p, uint8_t *rgb[3]){
  uint32_t rgb_mask = *reinterpret_cast<uint32_t*>(&p->rgb);
  *rgb[0] = (uint8_t)(rgb_mask >> 2 * BYTE);
  *rgb[1] = (uint8_t)(rgb_mask >> BYTE);
  *rgb[2] = (uint8_t)(rgb_mask);
}

int
main (int argc, char** argv)
{
  pcl::PointCloud<UberXYZ_RGB> cloud;
  cloud.points.resize (2);
  cloud.width = 2;
  cloud.height = 1;

  cloud.points[0].rgb = 1;
  cloud.points[1].rgb = 2;
  cloud.points[0].x = cloud.points[0].y = cloud.points[0].z = 0;
  cloud.points[1].x = cloud.points[1].y = cloud.points[1].z = 3;

  pcl::io::savePCDFile ("test.pcd", cloud);
}
