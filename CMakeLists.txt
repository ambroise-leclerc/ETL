cmake_minimum_required(VERSION 2.8.7)

project(ETL)

set(ETL_HEADERS include/etl/debug_policies.h include/etl/freestore.h include/etl/interrupts.h include/etl/ioports.h
                include/etl/metautils.h include/etl/watchdog.h)
                
set(LIBSTD_HEADERS libstd/include/chrono libstd/include/ratio)
set(LIBSTD_TESTS test/libstd/chrono.cpp test/libstd/memory.cpp)
                
set(ETL_ARCHITECTURE include/etl/architecture/MockCore.h)

set(HEADERS ${ETL_HEADERS} ${ETL_ARCHITECTURE} ${LIBSTD_HEADERS} test/MockDevice.h
            )
            
add_executable( TestsETL test/test.cpp test/SetClearTest.cpp test/InterruptsTest.cpp
                ${LIBSTD_TESTS}
                ${HEADERS} )

#target_compile_features(etltest PUBLIC cxx_binary_literals)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories(include 3rdParty test .)

