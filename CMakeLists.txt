cmake_minimum_required(VERSION 3.24)
project(EDL)

find_package(VTK REQUIRED)

add_executable(edl main.cpp)

target_link_libraries(edl PRIVATE ${VTK_LIBRARIES})

# vtk_module_autoinit is needed
vtk_module_autoinit(
  TARGETS edl
  MODULES ${VTK_LIBRARIES}
)