#include "ub_view.h"

int gui_states[GUIELEMENTS];
int gui_prev_states[GUIELEMENTS];

pcl::visualization::Camera ub_camera;
Eigen::Matrix4f ub_movement;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

pcl::PolygonMesh pcl_mesh;

vtkSmartPointer<vtkPolyData> vtk_mesh = vtkSmartPointer<vtkPolyData>::New();
vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
pcl::PointCloud<pcl::PointXYZ> cloud_mem;

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_rgb;
pcl::PointCloud<pcl::PointXYZRGB> cloud_rgbmem;
pcl::PointCloud<pcl::PointXYZRGB> cloud_rgbmem_copy;

pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb;

char *pcd_file = NULL;


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
    "\tThanks for using ub-view.\n"
  );
}

void initialize_view(){
  viewer->setBackgroundColor( 0.2, 0.3, 0.4);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0.0, 0.0, 2.0, 0, 0.2, 0, 0.0, 1.0, 0.0);
  viewer->registerKeyboardCallback(keyboard_handler, (void *)&viewer);
  viewer->getCameraParameters (ub_camera);
}

void update_view(){
  /*
  if(gui_states[]){ // RGB Enabled

  }else{ // Heat Map Enabled
    if(gui_states[])
      heat_map(cloud_rgbmem, gui_states[]);
  }
  */
}

void parse_arguments(int argc, char **argv){
  int c;

  while ((c = getopt (argc, argv, "hf:")) != -1){
    switch(c)
    {
        case 'h':
          print_help();
          exit(EXIT_SUCCESS);
          break;
        case 'f':
          pcd_file = optarg;
          break;
        default:
          break;
    }
  }
}

void run_loop(){
  while (!viewer->wasStopped ()){
    fflush(stdout); // Print any new messages

    update_view();

    viewer->spinOnce((int)(1000.0 / 60.0));

    boost::this_thread::sleep (boost::posix_time::microseconds ( 100));
  }
}

void initialize_geometry(){
  // Load the file
  pcl::io::loadPCDFile (pcd_file, cloud_rgbmem);

  // Fix up pointers & make some copies
  pcl::copyPointCloud(cloud_rgbmem, cloud_mem);
  pcl::copyPointCloud(cloud_rgbmem, cloud_rgbmem_copy);

  cloud = (pcl::PointCloud<pcl::PointXYZ>::ConstPtr) &cloud_mem;
  cloud_rgb = (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr) &cloud_rgbmem;

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);

  // Generate mesh from cloud
  cloud_to_mesh(cloud, pcl_mesh);
  pcl::VTKUtils::convertToVTK(pcl_mesh, vtk_mesh);
  color_vtkmesh(vtk_mesh);

  // Setup vtk components for rendering
  mapper->SetInput(vtk_mesh);
  actor->SetMapper(mapper);
}

int
main (int argc, char** argv)
{
  parse_arguments(argc, argv);

  if(pcd_file == NULL){
    printf("Sorry, no file provided. Use -f [file] argument.\n");
    return EXIT_FAILURE;
  }

  initialize_geometry();

  // Create pcl visualizer & add {cloud, mesh} to it
  viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("Uber (inter)View"));
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud_rgb, rgb, "Model Cloud");
  viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);

  initialize_view();

  // Start GUI
  // OpenCv locks up the thread waiting for keypress. Put it in its own.
  boost::thread gui_thread(setup_highgui);

  // Run
  run_loop();

  return EXIT_SUCCESS;
}
