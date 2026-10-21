pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

add_executable(camerad main.cc)
target_link_libraries(camerad PRIVATE PkgConfig::LIBCAMERA Threads::Threads)

# Header-only IPC helpers (ring.h, jsonsock.h) for the other daemons and tools.
add_library(roadsense_ipc INTERFACE)
target_include_directories(roadsense_ipc INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS camerad RUNTIME DESTINATION bin)
