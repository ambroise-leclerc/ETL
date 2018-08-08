cmake_minimum_required(VERSION 3.9.2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(ETL)
include_directories(include 3rdParty test .)

set(ETL_HEADERS include/etl/debug_policies.h include/etl/freestore.h include/etl/interrupts.h include/etl/ioports.h
                include/etl/metautils.h include/etl/watchdog.h)
                
set(LIBSTD_HEADERS  libstd/include/algorithm libstd/include/array libstd/include/bitset libstd/include/cassert libstd/include/cctype libstd/include/cerrno
        libstd/include/cfenv libstd/include/cfloat libstd/include/chrono libstd/include/climits libstd/include/clocale
        libstd/include/cstddef libstd/include/cstdint libstd/include/exception libstd/include/functional libstd/include/initializer_list
        libstd/include/iterator libstd/include/limits libstd/include/memory libstd/include/new libstd/include/queue libstd/include/random
        libstd/include/ratio libstd/include/stdexcept libstd/include/string libstd/include/thread libstd/include/tuple
        libstd/include/type_traits libstd/include/utility libstd/include/vector)
set(LIBSTD_TESTS test/libstd/bitset.cpp test/libstd/chrono.cpp test/libstd/limits.cpp test/libstd/memory.cpp test/libstd/tuple.cpp test/libstd/type_traits.cpp
				 test/libstd/queue.cpp test/libstd/string.cpp test/libstd/vector.cpp)
                
set(ETL_TESTS test/test.cpp test/SetClearTest.cpp test/InterruptsTest.cpp test/CircularBuffer.cpp)
set(ETL_ARCHITECTURE include/etl/architecture/MockCore.h)
set(ETL_DRIVERS include/etl/drivers/hd44780.h include/etl/drivers/tm1638.h include/etl/drivers/gfx/win32gdi.hpp)

set(HEADERS ${ETL_HEADERS} ${ETL_ARCHITECTURE} ${LIBSTD_HEADERS} ${ETL_DRIVERS} test/MockDevice.h
            )
            
add_executable( TestsETL ${ETL_TESTS}
                ${LIBSTD_TESTS}
                ${HEADERS} )

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT TestsETL)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(TestsETL Threads::Threads)

if (NOT(CMAKE_CXX_COMPILER MATCHES "MSVC"))
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-literal-suffix")
endif()
