cmake_minimum_required(VERSION 3.15)
project(ground_extractor)

find_package(PCL REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

find_path(GPERFTOOLS_INCLUDE_DIR gperftools/profiler.h)
find_library(GPERFTOOLS_LIBRARY_TEMP profiler REQUIRED)
if (NOT GPERFTOOLS_LIBRARY_TEMP)
  message(FATAL_ERROR "gperftools not found!")
endif()
add_library(gperftools STATIC IMPORTED)
set_target_properties(gperftools PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${GPERFTOOLS_INCLUDE_DIR})
set_target_properties(gperftools PROPERTIES IMPORTED_LOCATION ${GPERFTOOLS_LIBRARY_TEMP})
set(GPERFTOOLS_LIBRARY -Wl,--whole-archive gperftools -Wl,--no-whole-archive)


add_executable(ground_extractor
  extractor_test.cpp
)
target_link_libraries(ground_extractor
  dash::labelled_ground_extraction
  ${PCL_LIBRARIES}
  nlohmann_json::nlohmann_json  
  ${GPFERTOOLS_LIBRARY}
)