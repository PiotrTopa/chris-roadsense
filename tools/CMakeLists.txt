add_executable(ringdump ringdump.cc)
target_link_libraries(ringdump PRIVATE roadsense_ipc)
