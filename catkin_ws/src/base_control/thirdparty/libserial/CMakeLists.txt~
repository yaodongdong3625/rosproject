cmake_minimum_required(VERSION 2.8)
project(serial)
include_directories(
  include
)
AUX_SOURCE_DIRECTORY(src DIR_SRCS)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)
SET(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -Wall -std=c++11")
SET(CMAKE_C_FLAGS "$ENV{CFLAGS} -O3 -Wall")
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/../lib)
add_library(serial SHARED ${DIR_SRCS})
