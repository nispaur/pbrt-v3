cmake_minimum_required ( VERSION 2.8 )

project ( pbrt-kmeans )

add_library( kmgen src/kmgen.cpp )

target_include_directories (kmgen PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

