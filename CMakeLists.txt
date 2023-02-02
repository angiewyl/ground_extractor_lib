cmake_minimum_required(VERSION 3.15)
include_guard(GLOBAL)
project(dash_labelled_ground_extraction)

find_package(PCL CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

add_library(dash_labelled_ground_extraction
  src/ground_extractor.cpp
)
add_library(dash::labelled_ground_extraction ALIAS dash_labelled_ground_extraction)

target_link_libraries(dash_labelled_ground_extraction=
PUBLIC
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
PRIVATE
  spdlog::spdlog
)

target_include_directories(dash_labelled_ground_extraction
PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)