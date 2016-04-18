cmake_minimum_required(VERSION 2.8.7)

project(ETL)

add_executable(etltest include/etl.h 3rdParty/catch.hpp test/test.cpp)

#target_compile_features(etltest PUBLIC cxx_binary_literals)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z")

include_directories(include 3rdParty test)

