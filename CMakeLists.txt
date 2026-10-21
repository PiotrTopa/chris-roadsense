cmake_minimum_required(VERSION 3.16)
project(pocopilot CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Built on the phone itself (Snapdragon 845: Cortex-A75 + A55, ARMv8.2-A).
option(POCOPILOT_NATIVE "Tune for the build machine CPU (-mcpu=native)" ON)
include(CheckCXXCompilerFlag)
if(POCOPILOT_NATIVE)
  check_cxx_compiler_flag(-mcpu=native HAVE_MCPU_NATIVE)
  if(HAVE_MCPU_NATIVE)
    add_compile_options(-mcpu=native)
  endif()
endif()
add_compile_options(-Wall -Wextra -O3)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

add_subdirectory(camerad)
add_subdirectory(modeld)
# add_subdirectory(signd)
add_subdirectory(tools)
