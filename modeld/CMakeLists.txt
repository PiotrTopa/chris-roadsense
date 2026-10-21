# modeld: supercombo v0.9.7 on ONNX Runtime (CPU). Also builds tools/modeld_replay from the same library.
# Works as a subdirectory of the top-level project or standalone (cmake -S modeld), e.g. on a laptop
# with the onnxruntime-linux-x64 release tarball: -DONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-1.xx
cmake_minimum_required(VERSION 3.16)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(pocopilot_modeld CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
  add_compile_options(-Wall -Wextra -O3)
  find_package(Threads REQUIRED)
endif()
get_filename_component(POCOPILOT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)

set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "onnxruntime install prefix (include/, lib/)")
find_path(ORT_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}/include ${ONNXRUNTIME_ROOT}/include/onnxruntime
  PATHS /usr/include/onnxruntime /usr/local/include/onnxruntime)
find_library(ORT_LIBRARY onnxruntime HINTS ${ONNXRUNTIME_ROOT}/lib)
if(NOT ORT_INCLUDE_DIR OR NOT ORT_LIBRARY)
  message(FATAL_ERROR "onnxruntime not found (set ONNXRUNTIME_ROOT)")
endif()

add_library(modeld_core STATIC warp.cc supercombo.cc pipeline.cc)
target_include_directories(modeld_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${POCOPILOT_ROOT} ${ORT_INCLUDE_DIR})
target_link_libraries(modeld_core PUBLIC ${ORT_LIBRARY})
if(ONNXRUNTIME_ROOT)
  set_target_properties(modeld_core PROPERTIES INTERFACE_LINK_OPTIONS "-Wl,-rpath,${ONNXRUNTIME_ROOT}/lib")
endif()

add_executable(modeld main.cc)
target_link_libraries(modeld PRIVATE modeld_core Threads::Threads)

add_executable(modeld_replay ${POCOPILOT_ROOT}/tools/modeld_replay.cc)
target_link_libraries(modeld_replay PRIVATE modeld_core)

install(TARGETS modeld modeld_replay RUNTIME DESTINATION bin)
