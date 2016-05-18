cmake_minimum_required(VERSION 2.8.7)

project(ETL)

set(ETL_HEADERS include/etl/debug_policies.h include/etl/freestore.h include/etl/interrupts.h include/etl/ioports.h
                include/etl/metautils.h include/etl/watchdog.h include/etl/ostream.h)

set(ETL_LIBSTD libstd/include/algorithm libstd/include/array libstd/include/array_view libstd/include/blank libstd/include/cassert libstd/include/cctype
			   libstd/include/cerrno libstd/include/cfenv libstd/include/cfloat libstd/include/climits libstd/include/clocale libstd/include/cstddef
			   libstd/include/exception libstd/include/functional libstd/include/initializer_list libstd/include/iterator libstd/include/list
			   libstd/include/memory libstd/include/new libstd/include/ostream libstd/include/random libstd/include/stdexcept libstd/include/string
			   libstd/include/tuple libstd/include/type_traits libstd/include/utility libstd/include/vector)
                
set(ETL_ARCHITECTURE include/etl/architecture/MockCore.h)

set(HEADERS ${ETL_HEADERS} ${ETL_LIBSTD} ${ETL_ARCHITECTURE} test/MockDevice.h
            )
            
add_executable( Tests test/test.cpp test/SetClearTest.cpp test/InterruptsTest.cpp
                ${HEADERS} )

#target_compile_features(etltest PUBLIC cxx_binary_literals)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories(include 3rdParty test)

