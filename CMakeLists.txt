cmake_minimum_required(VERSION 2.8.7)

project(ETL)

set(HEADERS include/etl.h test/MockDevice.h  
            include/etl/architecture/MockCore.h )
add_executable( Tests test/test.cpp test/SetClearTest.cpp
                ${HEADERS} )

#target_compile_features(etltest PUBLIC cxx_binary_literals)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories(include 3rdParty test)

