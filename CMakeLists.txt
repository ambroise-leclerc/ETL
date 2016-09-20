cmake_minimum_required(VERSION 2.8.7)

project(ETL)

set(ETL_HEADERS include/etl/debug_policies.h include/etl/freestore.h include/etl/interrupts.h include/etl/ioports.h
                include/etl/metautils.h include/etl/watchdog.h)
                
set(LIBSTD_HEADERS libstd/include/chrono libstd/include/ratio libstd/include/limits libstd/include/memory libstd/include/utility
                   libstd/include/type_traits libstd/include/string libstd/include/vector libstd/include/tuple)
set(LIBSTD_TESTS test/libstd/chrono.cpp test/libstd/limits.cpp test/libstd/memory.cpp test/libstd/tuple.cpp test/libstd/type_traits.cpp)
                
set(ETL_TESTS test/test.cpp test/SetClearTest.cpp test/InterruptsTest.cpp test/CircularBuffer.cpp)
set(ETL_ARCHITECTURE include/etl/architecture/MockCore.h)

set(HEADERS ${ETL_HEADERS} ${ETL_ARCHITECTURE} ${LIBSTD_HEADERS} test/MockDevice.h
            )
            
add_executable( Tests ${ETL_TESTS}
                ${LIBSTD_TESTS}
                ${HEADERS} )

#target_compile_features(etltest PUBLIC cxx_binary_literals)
if(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

include_directories(include 3rdParty test .)

