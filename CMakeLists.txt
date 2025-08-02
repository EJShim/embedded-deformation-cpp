cmake_minimum_required(VERSION 3.2)
project(edl)
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# option(LIBIGL_USE_STATIC_LIBRARY     "Use libigl as static library" OFF)
add_subdirectory(libigl)

find_package(VTK REQUIRED)


add_executable(
  edl 
  main.cpp
  )

target_link_libraries(edl PRIVATE 
  ${VTK_LIBRARIES} 
  igl::core
  # igl_copyleft::tetgen
  )
# vtk_module_autoinit is needed
vtk_module_autoinit(
  TARGETS edl
  MODULES ${VTK_LIBRARIES}
)


if(WIN32)
    message(${VTK_PREFIX_PATH})
    file(GLOB VTK_DLLS ${VTK_PREFIX_PATH}/bin/*.dll)
    foreach(currentdll IN LISTS VTK_DLLS)
        message(${currentdll})
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
            COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
                ${currentdll}      # <--this is in-file
                $<TARGET_FILE_DIR:${PROJECT_NAME}>
                COMMENT "Copying dll: ${currentdll}")                 # <--this is out-file path
    endforeach()
endif(WIN32)
