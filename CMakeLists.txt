cmake_minimum_required(VERSION 3.0)

project(ETL)

add_executable(etltest include/etl.h 3rdParty/catch.hpp test/test.cpp)

target_compile_features(etltest PUBLIC cxx_binary_literals)

include_directories(include 3rdParty test)

