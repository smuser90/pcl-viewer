#include "ub_view.h"

int gui_states[GUIELEMENTS];
int gui_prev_states[GUIELEMENTS];

pcl::visualization::Camera ub_camera;
Eigen::Matrix4f ub_movement;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

pcl::PolygonMesh mesh;

bool mesh_colored = false;

vtkSmartPointer<vtkPolyData> raw_mesh = vtkSmartPointer<vtkPolyData>::New();

void print_help(){
  printf(
    "Uber (inter)View help:\n"
    "\tLoad PCD File:\n"
    "\t\t This program takes a single .pcd filename at startup.\n"
    "\t\t Use -f flag.\n\n"
    "\tMovement:\n"
    "\t\t This program provides translational movement along the camera\'s local xyz.\n"
    "\t\t \'[\' Move left.\n"
    "\t\t \'\\\' Move right.\n"
    "\t\t \']\' Move forward.\n"
    "\t\t \'Apostra\' Move backward.\n"
    "\t\t \'Period\' Move down.\n"
    "\t\t \'Period\' Move up.\n\n"
    "\tView:\n"
    "\t\t Click and drag to rotate the camera around the viewpoint.\n"
    "\t\t Use p,w,s keys for point, wire, and solid representations respectively.\n"
    "\t\t A geometry mesh is generated from the point cloud data on program start.\n\n"
    "\tColor:\n"
    "\t\t Use the trackbars to change cloud coloring options.\n"
    "\t\t Coloring style to use is picked top down.\n"
    "\t\t The first view enabled will be used.\n\n"
    "\tThanks for using ub-view!\n"
  );
}
int
main (int argc, char** argv)
{
  //pack_unpack_test();
  //binary_pcd_test();

  char *pcd_file = NULL;
  int char c;

  while ((c = getopt (argc, argv, "hf:")) != -1){
    switch c
    {
        case 'h':
          print_help();
          break;
        case 'f':
          pcd_file = optarg;
          break;
        default:
          break;
    }
  }

  if(pcd_file == NULL){
    printf("Sorry, no file provided. Use -f [file] argument.\n");
    return EXIT_FAILURE;
  }

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;   // Most functions seem to want a const ptr
  pcl::PointCloud<pcl::PointXYZ> cloud_mem;         // Allocate on stack for now.

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_rgb;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_rgbmem;

  pcl::io::loadPCDFile ("../model.pcd", cloud_mem);

  cloud = (pcl::PointCloud<pcl::PointXYZ>::ConstPtr) &cloud_mem;
  pcl::copyPointCloud(cloud_mem, cloud_rgbmem);

  heat_map(cloud_rgbmem);
  cloud_rgb = (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr) &cloud_rgbmem;

  cloud_to_mesh(cloud, mesh);

  pcl::VTKUtils::convertToVTK(mesh, raw_mesh);

  viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Uber View"));

  viewer->setBackgroundColor( 0.2, 0.3, 0.4);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud_rgb, rgb, "Model Cloud");

  vtkSmartPointer<vtkRenderWindow> vtkrender_window = viewer->getRenderWindow();
  vtkSmartPointer<vtkRenderer> vtkrenderer = vtkrender_window->GetRenderers()->GetFirstRenderer();

  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");

  double bounds[6];
  raw_mesh->GetBounds(bounds);

  vtkSmartPointer<vtkLookupTable> colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
  colorLookupTable->SetTableRange(bounds[2], bounds[3]);
  colorLookupTable->Build();

  for(int i = 0; i < raw_mesh->GetNumberOfPoints(); i++){
    double p[3];
    raw_mesh->GetPoint(i,p);

    double dcolor[3];
    colorLookupTable->GetColor(p[1], dcolor);

    unsigned char color[3];
    for(int j = 0; j < 3; j++)
      color[j] = static_cast<unsigned char>(255.0 * dcolor[j]);

    colors->InsertNextTupleValue(color);
  }

  raw_mesh->GetPointData()->SetScalars(colors);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput(raw_mesh);

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  vtkrenderer->AddActor(actor);

  fflush(stdout);

  //viewer->addPolygonMesh( mesh, "Model");
  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Model");

  viewer->addCoordinateSystem(0.1);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0.0, 0.0, 2.0, 0, 0.2, 0, 0.0, 1.0, 0.0);
  viewer->registerKeyboardCallback(keyboard_handler, (void *)&viewer);
  viewer->getCameraParameters (ub_camera);

  boost::thread gui_thread(setup_highgui);

  while (!viewer->wasStopped ()){
    viewer->setBackgroundColor( gui_states[1] / 255.0, gui_states[2] / 255.0, gui_states[3] / 255.0);
    viewer->spinOnce((int)(1000.0 / 60.0));


    //printf("GUI States: %d\t%d\t%d\t%d", enable_tbar, red_tbar, green_tbar, blue_tbar);
    boost::this_thread::sleep (boost::posix_time::microseconds ( 1000));
  }

  return EXIT_SUCCESS;
}
