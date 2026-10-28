cmake_minimum_required(VERSION 3.25)
project(ETLStdConsumer LANGUAGES C CXX)
find_package(ETLStd 0.2 CONFIG REQUIRED)
set(ETLSTD_TEST_TARGET ETL::StdMinimal CACHE STRING "ETLStd profile to consume")
add_executable(consumer main.cpp helper.c)
target_link_libraries(consumer PRIVATE ${ETLSTD_TEST_TARGET})
target_compile_options(consumer PRIVATE "$<$<COMPILE_LANGUAGE:C>:-std=c11;-Werror>")
target_link_options(consumer PRIVATE -Wl,--gc-sections)

# A reference binary with the same compiler flags, but no profile identity, measures its cost.
add_executable(unprofiled main.cpp helper.c)
target_link_libraries(unprofiled PRIVATE ETL::StdHeaders)
if (ETLSTD_TEST_TARGET STREQUAL "ETL::StdMinimal")
    set(profile_flags -fno-exceptions -fno-rtti)
else()
    set(profile_flags -fexceptions -frtti)
endif()
target_compile_options(unprofiled PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-std=c++23;-ffreestanding;${profile_flags};-nostdinc++>"
    "$<$<COMPILE_LANGUAGE:C>:-std=c11;-Werror>")
target_link_options(unprofiled PRIVATE -nostdlib++ -Wl,--gc-sections)
