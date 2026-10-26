# camerad-replay: recorded ride (MJPEG file) -> frame ring, looping. Needs libjpeg (libjpeg-turbo).
add_executable(camerad-replay main.cc)
target_link_libraries(camerad-replay PRIVATE roadsense_ipc JPEG::JPEG)
install(TARGETS camerad-replay RUNTIME DESTINATION bin)
