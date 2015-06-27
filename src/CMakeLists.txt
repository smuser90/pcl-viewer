cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project(Uber_Viewer)

include(FindPkgConfig)
pkg_check_modules(OPENNI openni)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common io kdtree visualization search features surface)
find_package(OpenCV REQUIRED)
find_package(VTK REQUIRED)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_ADDITIONAL_VERSIONS "1.44" "1.44.0")

find_package(Boost COMPONENTS thread date_time program_options filesystem system REQUIRED)
find_package(Threads REQUIRED)

include(${VTK_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions(${BOOST_THREAD_USE_LIB})

add_executable(ub-view ${SUBSYS_NAME} ub_gui.cpp ub_utils.cpp ub_tests.cpp ub_view.cpp ub_controls.cpp)

target_link_libraries(ub-view ${PCL_LIBRARIES} ${Boost_LIBRARIES}
  libvtkCommon.so libvtkFiltering.so libvtkRendering.so ${OpenCV_LIBS} ${OPENNI_LIBRARIES} ${VTK_LIBRARIES})
