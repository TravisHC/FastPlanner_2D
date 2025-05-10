cmake_minimum_required (VERSION 3.5)

project (FastPlanner)


set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -std=c++17")

include_directories(trajopt_bspline/include/grad_replanner)

add_subdirectory(localmap_esdf)
add_subdirectory(trajopt_bspline)

