# laned: lane lines for the display, classical analysis of the luma plane (no model, no GPU, no dependency),
# and tools/laned_replay. Works as a subdirectory or standalone (cmake -S laned).
cmake_minimum_required(VERSION 3.16)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(roadsense_laned CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
  add_compile_options(-Wall -Wextra -O3)
endif()
get_filename_component(ROADSENSE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)

add_library(laned_core STATIC lanefind.cc)
target_include_directories(laned_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${ROADSENSE_ROOT})

add_executable(laned main.cc)
target_link_libraries(laned PRIVATE laned_core)

add_executable(laned_replay ${ROADSENSE_ROOT}/tools/laned_replay.cc)
target_link_libraries(laned_replay PRIVATE laned_core)

install(TARGETS laned laned_replay RUNTIME DESTINATION bin)
