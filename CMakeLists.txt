cmake_minimum_required (VERSION 3.0)
SET (CMAKE_CXX_COMPILER "/usr/bin/clang++")
SET (CMAKE_CXX_FLAGS "-Wall")

project(KDTree)

include_directories(include)

add_library(kdtree src/kdtree.cpp)

#target_include_directories(kdtree PUBLIC include/)

add_executable(demo1 src/test.cpp)

target_link_libraries(demo1 kdtree)

install(TARGETS kdtree DESTINATION /usr/lib)
