# signd: MTSD traffic-sign detector + classifier + SORT tracker on ONNX Runtime (CPU).
# Also builds tools/signd_replay from the same library.
# Works as a subdirectory of the top-level project or standalone (cmake -S signd), e.g. on a laptop
# with the onnxruntime-linux-x64 release tarball: -DONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-1.xx
# signd itself only needs ONNX Runtime; signd_replay decodes JPEG inputs if libjpeg is found
# (JPEG_INCLUDE_DIR / JPEG_LIBRARY can point at an unpacked -dev package), PPM/NV12 otherwise.
cmake_minimum_required(VERSION 3.16)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(pocopilot_signd CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
  add_compile_options(-Wall -Wextra -O3)
  include(CheckCXXCompilerFlag)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
    check_cxx_compiler_flag(-mcpu=native HAVE_MCPU_NATIVE)
  endif()
  if(HAVE_MCPU_NATIVE)
    add_compile_options(-mcpu=native)
  endif()
  find_package(Threads REQUIRED)
endif()
get_filename_component(POCOPILOT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)

set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "onnxruntime install prefix (include/, lib/)")
find_path(SIGND_ORT_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}/include ${ONNXRUNTIME_ROOT}/include/onnxruntime
  PATHS /usr/include/onnxruntime /usr/local/include/onnxruntime)
find_library(SIGND_ORT_LIBRARY onnxruntime HINTS ${ONNXRUNTIME_ROOT}/lib)
if(NOT SIGND_ORT_INCLUDE_DIR OR NOT SIGND_ORT_LIBRARY)
  message(FATAL_ERROR "onnxruntime not found (set ONNXRUNTIME_ROOT)")
endif()

# camerad/ holds the header-only IPC contract (ring.h, jsonsock.h) and the NV12 JPEG encoder.
add_library(signd_core STATIC yolo.cc tracker.cc output.cc)
target_include_directories(signd_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${POCOPILOT_ROOT}/camerad
  ${SIGND_ORT_INCLUDE_DIR})
target_link_libraries(signd_core PUBLIC ${SIGND_ORT_LIBRARY})
if(ONNXRUNTIME_ROOT)
  set_target_properties(signd_core PROPERTIES INTERFACE_LINK_OPTIONS "-Wl,-rpath,${ONNXRUNTIME_ROOT}/lib")
endif()

add_executable(signd main.cc)
target_link_libraries(signd PRIVATE signd_core Threads::Threads)

add_executable(signd_replay ${POCOPILOT_ROOT}/tools/signd_replay.cc)
target_link_libraries(signd_replay PRIVATE signd_core)
find_package(JPEG QUIET)
if(JPEG_FOUND)
  target_compile_definitions(signd_replay PRIVATE SIGND_HAVE_LIBJPEG)
  target_include_directories(signd_replay PRIVATE ${JPEG_INCLUDE_DIRS})
  target_link_libraries(signd_replay PRIVATE ${JPEG_LIBRARIES})
else()
  message(STATUS "signd_replay: libjpeg not found, JPEG input disabled (PPM/PGM/NV12 only)")
endif()

install(TARGETS signd signd_replay RUNTIME DESTINATION bin)
