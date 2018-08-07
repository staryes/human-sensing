
cmake_minimum_required(VERSION 3.5)

project(human-sensing)

find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribOptions)
include(ICUBcontribHelpers)

#add_subdirectory(faceLandmarks)
add_subdirectory(yarpOpenFace)

icubcontrib_add_uninstall_target()
