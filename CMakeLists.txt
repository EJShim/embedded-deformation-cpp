cmake_minimum_required(VERSION 3.2)
project(EDL)
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# option(LIBIGL_USE_STATIC_LIBRARY     "Use libigl as static library" OFF)
add_subdirectory(libigl)

find_package(VTK REQUIRED)


add_executable(
  edl 
  main.cpp
  )

target_link_libraries(edl PRIVATE ${VTK_LIBRARIES} igl::core)
# vtk_module_autoinit is needed
vtk_module_autoinit(
  TARGETS edl
  MODULES ${VTK_LIBRARIES}
)