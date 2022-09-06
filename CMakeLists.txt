cmake_minimum_required(VERSION 3.2)
project(EDL)

set(CMAKE_INCLUDE_CURRENT_DIR ON)


find_package(VTK REQUIRED)

add_executable(
  edl 
  main.cpp
  fast_marching.cpp
  deformation_graph.cpp
  )

target_link_libraries(edl PRIVATE ${VTK_LIBRARIES})
# vtk_module_autoinit is needed
vtk_module_autoinit(
  TARGETS edl
  MODULES ${VTK_LIBRARIES}
)