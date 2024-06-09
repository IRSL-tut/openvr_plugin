cmake_minimum_required(VERSION 3.10)

if(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
  # Build as a master project
  Message ("Build : OpenVRPlugin / separated from choreonoid/ext")
  project(OpenVRPlugin)
  find_package(Choreonoid REQUIRED)

  add_subdirectory(src)
else ()
  # Build at choreonoid/ext
  Message ("Build : OpenVRPlugin / under choreonoid/ext")
  add_subdirectory(src)
endif()
