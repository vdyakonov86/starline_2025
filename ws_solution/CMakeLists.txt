cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcd_detection)

find_package(PCL 1.2 REQUIRED)

set(SRC src/PcdLoader.cpp src/PcdVisualizer.cpp src/PcdFilter.cpp)
set(INCLUDE include/PcdLoader.hpp include/PcdVisualizer.hpp include/PcdFilter.hpp)

add_executable (main main.cpp ${SRC} ${INCLUDE})

target_include_directories(main PRIVATE include) # чтобы main видел твои .hpp
target_link_libraries (main ${PCL_LIBRARIES})