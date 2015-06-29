#include "ub_view.h"

int gui_states[GUIELEMENTS];
int gui_prev_states[GUIELEMENTS];

pcl::visualization::Camera ub_camera;
Eigen::Matrix4f ub_movement;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

int meshed;
pcl::PolygonMesh pcl_mesh;

vtkSmartPointer<vtkPolyData> vtk_mesh = vtkSmartPointer<vtkPolyData>::New();
vtkSmartPointer<vtkUnsignedCharArray> vtk_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
vtkSmartPointer<vtkLookupTable> vtk_colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
pcl::PointCloud<pcl::PointXYZ> cloud_mem;

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_rgb;
pcl::PointCloud<pcl::PointXYZRGB> cloud_rgbmem;
pcl::PointCloud<pcl::PointXYZRGB> cloud_rgbmem_copy;

pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb;

char *pcd_file = NULL;


void print_help(bool pcl = false){

    if(pcl)
      printf("\n------------------------------ More --------------------------------");
    else
      printf("Uber (inter)View help:\n");

    printf(
    "\tLoad PCD File:\n"
    "\t\t This program takes a single .pcd filename at startup.\n"
    "\t\t Use -f flag.\n\n"
    "\tSave PCD snapshot:\n"
    "\t\t  'z' Save the current view to snapshot file in `pwd`."
    "\tMovement:\n"
    "\t\t This program provides translational movement along the camera\'s local xyz.\n"
    "\t\t \'[\' Move left.\n"
    "\t\t \'\\\' Move right.\n"
    "\t\t \']\' Move forward.\n"
    "\t\t \'Apostra\' Move backward.\n"
    "\t\t \'Period\' Move down.\n"
    "\t\t \'/\' Move up.\n\n"
    "\tView:\n"
    "\t\t Click and drag to rotate the camera around the viewpoint.\n"
    "\t\t Use p,w,s keys for point, wire, and solid representations respectively.\n"
    "\tColor:\n"
    "\t\t Use the trackbars to change cloud coloring options.\n"
    "\t\t Coloring style to use is picked top down.\n"
    "\t\t The first view enabled will be used.\n\n"
    "\tMeshing:\n"
    "\t\t A geometry mesh is generated from the point cloud data on program start.\n"
    "\t\t To reveal the mesh, choose an rgb value then press the 'm'.\n"
    "\tThanks for using ub-view.\n"
  );

  if(pcl)
    printf("\n------------------------------ More --------------------------------");
}

void reset_view(){
  double bounds[6];
  vtk_mesh->GetBounds(bounds);
  viewer->setCameraPosition(bounds[1]*1.2, 0, bounds[5] * 1.2, 0, 0, 0, 0.0, 1.0, 0.0);
}

void initialize_view(){
  double bounds[6];
  vtk_mesh->GetBounds(bounds);

  viewer->setBackgroundColor( 0.2, 0.3, 0.4);
  viewer->initCameraParameters();
  viewer->setCameraPosition(bounds[1]*1.2, 0, bounds[5] * 1.2, 0, 0, 0, 0.0, 1.0, 0.0);
  viewer->registerKeyboardCallback(keyboard_handler, (void *)&viewer);
  viewer->getCameraParameters (ub_camera);
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

bool heat_map_on(){
  return ((gui_states[GUIHEATMAP] != gui_prev_states[GUIHEATMAP]) && (gui_states[GUIHEATMAP] == 1)) || (gui_states[GUIHEATMAP] == 1 && gui_states[GUIXYZ] != gui_prev_states[GUIXYZ]);
}

bool color_changed(){
  return (gui_states[GUIRED] != gui_prev_states[GUIRED]) || (gui_states[GUIGREEN] != gui_prev_states[GUIGREEN]) || (gui_states[GUIBLUE] != gui_prev_states[GUIBLUE]);

}

void refresh_visualizer(){
  viewer->removePointCloud("Model Cloud");

  rgb = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud_rgb);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud_rgb, rgb, "Model Cloud");

  if(meshed && meshed < 3){
    // Setup vtk components for rendering
    mapper->SetInput(vtk_mesh);
    actor->SetMapper(mapper);
    viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
    meshed++;
  }
}

void mesh_step2(){
  if(meshed && meshed < 2){
    vtk_colors->SetNumberOfComponents(3);
    vtk_colors->SetName("Colors");
    meshed++;
  }
}
void update_view(){

  if(heat_map_on() ){

    mesh_step2();
    heat_map(cloud_rgbmem, gui_states[GUIXYZ], vtk_mesh);
    refresh_visualizer();

  }else{ // Heat Map Enabled
    if(color_changed()){
      mesh_step2();

      uint8_t rgb[3];
      rgb[0] = gui_states[GUIRED];
      rgb[1] = gui_states[GUIGREEN];
      rgb[2] = gui_states[GUIBLUE];

      color_cloud(cloud_rgbmem, rgb);
      color_vtkmesh(vtk_mesh, rgb);
      refresh_visualizer();
    }
  }

  for(int x = 0; x < GUIELEMENTS; x++)
    gui_prev_states[x] = gui_states[x];
}

void run_loop(){
  for(int x = 0; x < GUIELEMENTS; x++)
    gui_prev_states[x] = gui_states[x];

  while (!viewer->wasStopped ()){
    fflush(stdout); // Print any new messages

    update_view();

    viewer->spinOnce((int)(1000.0 / 60.0));

    boost::this_thread::sleep (boost::posix_time::microseconds ( 100));
  }
}

void save_file(){
  time_t rawtime;
  tm* timeinfo;
  char tstring [80];
  char filename [150];
  time(&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(tstring,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
  sprintf(filename, "ub-view-snapshot-%s.pcd", tstring);

  int retval = pcl::io::savePCDFile (filename, cloud_rgbmem);
  if(!retval)
    printf("Saved a snapshot to file: %s\n", filename);
  else
    printf("Error: A snapshot file could not be saved.\n");
}

void mesh_cloud(){
  if(!meshed) meshed++;
}

void initialize_geometry(){
  // Load the file
  pcl::io::loadPCDFile (pcd_file, cloud_rgbmem);

  // Fix up pointers & make some copies
  pcl::copyPointCloud(cloud_rgbmem, cloud_mem);
  pcl::copyPointCloud(cloud_rgbmem, cloud_rgbmem_copy);

  cloud = (pcl::PointCloud<pcl::PointXYZ>::ConstPtr) &cloud_mem;
  cloud_rgb = (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr) &cloud_rgbmem;

  rgb = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud_rgb);

  // Generate mesh from cloud
  meshed = 0;
  cloud_to_mesh(cloud, pcl_mesh);
  pcl::VTKUtils::convertToVTK(pcl_mesh, vtk_mesh);
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

  initialize_view();

  // Start GUI
  // OpenCv locks up the thread waiting for keypress. Put it in its own.
  boost::thread gui_thread(setup_highgui);

  // Run
  run_loop();

  return EXIT_SUCCESS;
}
