# modeld: supercombo v0.9.7 on ONNX Runtime (CPU) and optionally ncnn (Vulkan). Also builds
# tools/modeld_replay from the same library.
# Works as a subdirectory of the top-level project or standalone (cmake -S modeld), e.g. on a laptop
# with the onnxruntime-linux-x64 release tarball: -DONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-1.xx
#
# ncnn backend (--backend ncnn): -DROADSENSE_NCNN=ON plus either
#   -DNCNN_DIR=/path/to/ncnn   an ncnn source tree built in <NCNN_DIR>/build (scripts/build-ncnn.sh):
#                              headers from src/ and build/src/, static libncnn.a + bundled glslang libs
#   (no NCNN_DIR)              an installed ncnn found by find_package(ncnn) (CMAKE_PREFIX_PATH / ncnn_DIR)
cmake_minimum_required(VERSION 3.16)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(roadsense_modeld CXX)
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
find_path(ORT_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}/include ${ONNXRUNTIME_ROOT}/include/onnxruntime
  PATHS /usr/include/onnxruntime /usr/local/include/onnxruntime)
find_library(ORT_LIBRARY onnxruntime HINTS ${ONNXRUNTIME_ROOT}/lib)
if(NOT ORT_INCLUDE_DIR OR NOT ORT_LIBRARY)
  message(FATAL_ERROR "onnxruntime not found (set ONNXRUNTIME_ROOT)")
endif()

add_library(modeld_core STATIC warp.cc supercombo.cc pipeline.cc ort_backend.cc)
target_include_directories(modeld_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${ROADSENSE_ROOT} ${ORT_INCLUDE_DIR})
target_link_libraries(modeld_core PUBLIC ${ORT_LIBRARY})

option(ROADSENSE_NCNN "Build the ncnn (Vulkan) inference backend" OFF)
set(NCNN_DIR "$ENV{NCNN_DIR}" CACHE PATH "ncnn source tree with a build in <NCNN_DIR>/build (empty: find_package(ncnn))")
if(ROADSENSE_NCNN)
  target_sources(modeld_core PRIVATE ncnn_backend.cc)
  target_compile_definitions(modeld_core PUBLIC ROADSENSE_HAVE_NCNN)
  if(NCNN_DIR)
    set(_nb ${NCNN_DIR}/build)
    if(NOT EXISTS ${_nb}/src/libncnn.a)
      message(FATAL_ERROR "NCNN_DIR=${NCNN_DIR}: ${_nb}/src/libncnn.a not found (scripts/build-ncnn.sh)")
    endif()
    find_package(OpenMP REQUIRED)
    find_package(Threads REQUIRED)
    target_include_directories(modeld_core SYSTEM PRIVATE ${NCNN_DIR}/src ${_nb}/src)
    # Static link order: ncnn, then glslang (SPIRV -> glslang -> MachineIndependent -> OSDependent,
    # GenericCodeGen), grouped so the order between the glslang archives does not matter.
    target_link_libraries(modeld_core PUBLIC
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
    message(STATUS "modeld: ncnn backend from ${NCNN_DIR}")
  else()
    find_package(ncnn REQUIRED)
    target_link_libraries(modeld_core PUBLIC ncnn)
    message(STATUS "modeld: ncnn backend from installed ncnn (${ncnn_DIR})")
  endif()
endif()
if(ONNXRUNTIME_ROOT)
  set_target_properties(modeld_core PROPERTIES INTERFACE_LINK_OPTIONS "-Wl,-rpath,${ONNXRUNTIME_ROOT}/lib")
endif()

add_executable(modeld main.cc)
target_link_libraries(modeld PRIVATE modeld_core Threads::Threads)

add_executable(modeld_replay ${ROADSENSE_ROOT}/tools/modeld_replay.cc)
target_link_libraries(modeld_replay PRIVATE modeld_core)

install(TARGETS modeld modeld_replay RUNTIME DESTINATION bin)
