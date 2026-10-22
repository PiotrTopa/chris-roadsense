# Header-only IPC helpers (ring.h, jsonsock.h) for the other daemons and tools.
add_library(roadsense_ipc INTERFACE)
target_include_directories(roadsense_ipc INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# camerad (libcamera). Without libcamera (e.g. a laptop build for camerad-dvr) it is skipped.
pkg_check_modules(LIBCAMERA IMPORTED_TARGET libcamera)
if(NOT LIBCAMERA_FOUND)
  message(WARNING "libcamera not found: camerad is not built")
  return()
endif()
add_executable(camerad main.cc)
target_link_libraries(camerad PRIVATE PkgConfig::LIBCAMERA Threads::Threads)
install(TARGETS camerad RUNTIME DESTINATION bin)
