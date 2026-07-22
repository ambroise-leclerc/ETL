cmake_minimum_required(VERSION 3.25...4.2)

project(ETL LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ETL_BUILD_HOST_TESTS "Build the host-side Catch test suite" ON)
option(ETL_BUILD_AVR_SMOKE "Build the AVR smoke example with avr-g++" OFF)

set(ETL_AVR_MCU "atmega328p" CACHE STRING "Target MCU for AVR smoke builds")
set(ETL_AVR_F_CPU "16000000UL" CACHE STRING "F_CPU definition for AVR smoke builds")

set(ETL_HEADERS include/etl/debug_policies.h include/etl/freestore.h include/etl/interrupts.h include/etl/ioports.h
                include/etl/metautils.h include/etl/watchdog.h)
                
set(LIBSTD_HEADERS  libstd/include/algorithm libstd/include/array libstd/include/bitset libstd/include/cassert libstd/include/cctype libstd/include/cerrno
        libstd/include/cfenv libstd/include/cfloat libstd/include/chrono libstd/include/climits libstd/include/clocale
        libstd/include/cstddef libstd/include/cstdint libstd/include/exception libstd/include/functional libstd/include/initializer_list
        libstd/include/iterator libstd/include/limits libstd/include/memory libstd/include/new libstd/include/queue libstd/include/random
        libstd/include/ratio libstd/include/stdexcept libstd/include/string libstd/include/thread libstd/include/tuple
        libstd/include/type_traits libstd/include/utility libstd/include/vector)
set(LIBSTD_TESTS test/libstd/array.cpp test/libstd/bitset.cpp test/libstd/chrono.cpp test/libstd/exception.cpp test/libstd/functional.cpp
                 test/libstd/limits.cpp test/libstd/memory.cpp test/libstd/random.cpp test/libstd/thread.cpp test/libstd/tuple.cpp
                 test/libstd/type_traits.cpp test/libstd/queue.cpp test/libstd/string.cpp test/libstd/string_view.cpp test/libstd/vector.cpp)
                 
set(ETL_TESTS test/test.cpp test/SetClearTest.cpp test/InterruptsTest.cpp test/CircularBuffer.cpp)
set(ETL_ARCHITECTURE include/etl/architecture/MockCore.h)
set(ETL_DRIVERS include/etl/drivers/hd44780.h include/etl/drivers/tm1638.h include/etl/drivers/gfx/win32gdi.hpp)

set(HEADERS ${ETL_HEADERS} ${ETL_ARCHITECTURE} ${LIBSTD_HEADERS} ${ETL_DRIVERS} test/MockDevice.h)

add_library(etl INTERFACE)
target_include_directories(etl
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
target_compile_features(etl INTERFACE cxx_std_23)

if (ETL_BUILD_HOST_TESTS)
    add_executable(TestsETL
        ${ETL_TESTS}
        ${LIBSTD_TESTS}
        ${HEADERS}
    )

    target_link_libraries(TestsETL PRIVATE etl)
    target_include_directories(TestsETL PRIVATE 3rdParty test)
    target_compile_features(TestsETL PRIVATE cxx_std_23)

    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    target_link_libraries(TestsETL PRIVATE Threads::Threads)

    if (NOT(CMAKE_CXX_COMPILER MATCHES "MSVC"))
        target_compile_options(TestsETL PRIVATE -Wno-literal-suffix)
    endif()

    include(CTest)
    if (BUILD_TESTING)
        add_test(NAME TestsETL COMMAND TestsETL ~[libstd])
        add_test(NAME TestsETL-libstd COMMAND TestsETL [libstd])
    endif()

    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT TestsETL)
endif()

if (ETL_BUILD_AVR_SMOKE)
    if (NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "avr")
        message(FATAL_ERROR "ETL_BUILD_AVR_SMOKE requires an AVR toolchain. Configure with -DCMAKE_TOOLCHAIN_FILE=cmake/avr-gcc-toolchain.cmake.")
    endif()

    function(etl_add_avr_smoke_target target_name source_file)
        add_executable(${target_name}
            ${source_file}
            ${HEADERS}
        )

        target_link_libraries(${target_name} PRIVATE etl)
        target_compile_features(${target_name} PRIVATE cxx_std_23)
        target_compile_definitions(${target_name} PRIVATE F_CPU=${ETL_AVR_F_CPU})
        target_compile_options(${target_name} PRIVATE -Os -ffunction-sections -fdata-sections)
        target_link_options(${target_name} PRIVATE -Wl,--gc-sections)
        set_target_properties(${target_name} PROPERTIES OUTPUT_NAME "${target_name}" SUFFIX ".elf")

        if (CMAKE_OBJCOPY)
            add_custom_command(
                TARGET ${target_name} POST_BUILD
                COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom $<TARGET_FILE:${target_name}> ${CMAKE_CURRENT_BINARY_DIR}/${target_name}.hex
                BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/${target_name}.hex
                COMMENT "Generating Intel HEX image for ${target_name}"
            )
        endif()
    endfunction()

    etl_add_avr_smoke_target(etl_avr_smoke examples/avr/smoke.cpp)
    etl_add_avr_smoke_target(etl_avr_libstd_smoke examples/avr/libstd_smoke.cpp)
endif()
