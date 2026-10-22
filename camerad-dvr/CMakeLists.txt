# camerad-dvr: Huiying/Buildwin USB DVR camera (1902:3241) -> frame ring. Needs libjpeg (libjpeg-turbo).
add_executable(camerad-dvr main.cc)
target_link_libraries(camerad-dvr PRIVATE roadsense_ipc JPEG::JPEG)
install(TARGETS camerad-dvr RUNTIME DESTINATION bin)
