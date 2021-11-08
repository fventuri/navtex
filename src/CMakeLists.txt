add_library(libnavtex SHARED fftfilt.cxx navtex_rx.cpp)

add_executable(navtex_rx_from_file navtex_rx_from_file.cpp)
target_link_libraries(navtex_rx_from_file libnavtex)
