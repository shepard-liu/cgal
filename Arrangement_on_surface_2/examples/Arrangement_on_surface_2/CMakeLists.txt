# Created by the script cgal_create_cmake_script
# This is the CMake script for compiling a CGAL application.

cmake_minimum_required(VERSION 3.12...3.31)
project(Arrangement_on_surface_2_Examples)

find_package(CGAL REQUIRED COMPONENTS Core OPTIONAL_COMPONENTS Qt6)
find_package(TBB QUIET)
include(CGAL_TBB_support)

if(NOT TARGET CGAL::TBB_support)
  message(STATUS "NOTICE: TBB support is not available. Parallelization will be disabled.")
endif()

if(NOT CGAL_Qt6_FOUND)
  message(STATUS "NOTICE: Qt6 support is not available. The examples using the Basic_viewer will not be compiled.")
endif()

# create a target per cppfile
file(
  GLOB cppfiles
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

foreach(cppfile ${cppfiles})
  create_single_source_cgal_program("${cppfile}")
endforeach()

foreach(target
  circles
  circular_arcs
  draw_arr
  draw_agas
  ellipses
  hyperbolas
  linear_conics
  parabolas
  polylines
  rational_functions
  spherical_insert
  spherical_overlay
  unbounded_non_intersecting)
  if(CGAL_Qt6_FOUND)
    target_link_libraries(${target} PRIVATE CGAL::CGAL_Basic_viewer)
  endif()

  if(TARGET CGAL::TBB_support)
    target_link_libraries(${target} PRIVATE CGAL::TBB_support)
  endif()
endforeach(target)