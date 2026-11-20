# envd: zero-shot scene/environment analyser (CLIP-type image encoder on ONNX Runtime, optionally
# ncnn/Vulkan) and tools/envd_replay. Same ORT / ncnn discovery as modeld/CMakeLists.txt; works as a
# subdirectory or standalone (cmake -S envd -DONNXRUNTIME_ROOT=...).
cmake_minimum_required(VERSION 3.16)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(roadsense_envd CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
  add_compile_options(-Wall -Wextra -O3)
  find_package(Threads REQUIRED)
endif()
get_filename_component(ROADSENSE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)

set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "onnxruntime install prefix (include/, lib/)")
find_path(ENVD_ORT_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}/include ${ONNXRUNTIME_ROOT}/include/onnxruntime
  PATHS /usr/include/onnxruntime /usr/local/include/onnxruntime)
find_library(ENVD_ORT_LIBRARY onnxruntime HINTS ${ONNXRUNTIME_ROOT}/lib)
if(NOT ENVD_ORT_INCLUDE_DIR OR NOT ENVD_ORT_LIBRARY)
  message(FATAL_ERROR "onnxruntime not found (set ONNXRUNTIME_ROOT)")
endif()

add_library(envd_core STATIC classes.cc prep.cc pipeline.cc output.cc ort_encoder.cc)
target_include_directories(envd_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${ROADSENSE_ROOT} ${ENVD_ORT_INCLUDE_DIR})
target_link_libraries(envd_core PUBLIC ${ENVD_ORT_LIBRARY} Threads::Threads)

option(ROADSENSE_NCNN "Build the ncnn (Vulkan) inference backend" OFF)
set(NCNN_DIR "$ENV{NCNN_DIR}" CACHE PATH "ncnn source tree with a build in <NCNN_DIR>/build (empty: find_package(ncnn))")
if(ROADSENSE_NCNN)
  target_sources(envd_core PRIVATE ncnn_encoder.cc)
  target_compile_definitions(envd_core PUBLIC ROADSENSE_HAVE_NCNN)
  if(NCNN_DIR)
    set(_nb ${NCNN_DIR}/build)
    if(NOT EXISTS ${_nb}/src/libncnn.a)
      message(FATAL_ERROR "NCNN_DIR=${NCNN_DIR}: ${_nb}/src/libncnn.a not found (scripts/build-ncnn.sh)")
    endif()
    find_package(OpenMP REQUIRED)
    target_include_directories(envd_core SYSTEM PRIVATE ${NCNN_DIR}/src ${_nb}/src)
    target_link_libraries(envd_core PUBLIC
      ${_nb}/src/libncnn.a
      -Wl,--start-group
      ${_nb}/glslang/SPIRV/libSPIRV.a
      ${_nb}/glslang/glslang/libglslang.a
      ${_nb}/glslang/glslang/libMachineIndependent.a
      ${_nb}/glslang/glslang/libGenericCodeGen.a
      ${_nb}/glslang/glslang/OSDependent/Unix/libOSDependent.a
      ${_nb}/glslang/glslang/libglslang-default-resource-limits.a
      -Wl,--end-group
      OpenMP::OpenMP_CXX Threads::Threads ${CMAKE_DL_LIBS})
    message(STATUS "envd: ncnn backend from ${NCNN_DIR}")
  else()
    find_package(ncnn REQUIRED)
    target_link_libraries(envd_core PUBLIC ncnn)
    message(STATUS "envd: ncnn backend from installed ncnn (${ncnn_DIR})")
  endif()
endif()
if(ONNXRUNTIME_ROOT)
  set_target_properties(envd_core PROPERTIES INTERFACE_LINK_OPTIONS "-Wl,-rpath,${ONNXRUNTIME_ROOT}/lib")
endif()

add_executable(envd main.cc)
target_link_libraries(envd PRIVATE envd_core)

add_executable(envd_replay ${ROADSENSE_ROOT}/tools/envd_replay.cc)
target_link_libraries(envd_replay PRIVATE envd_core)
find_package(JPEG QUIET)
if(JPEG_FOUND)
  target_compile_definitions(envd_replay PRIVATE ENVD_HAVE_LIBJPEG)
  target_include_directories(envd_replay PRIVATE ${JPEG_INCLUDE_DIRS})
  target_link_libraries(envd_replay PRIVATE ${JPEG_LIBRARIES})
else()
  message(STATUS "envd_replay: libjpeg not found, JPEG input disabled (--nv12 only)")
endif()

install(TARGETS envd envd_replay RUNTIME DESTINATION bin)

# trigger_test: the scene-change trigger (trigger.h) on synthetic frames; --nv12 dumps thumbnails for the
# parity check against tools/envd/rate_sim.py (lane L48-E)
add_executable(trigger_test trigger_test.cc)
target_include_directories(trigger_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
