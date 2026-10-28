if (NOT TARGET ETL::StdMinimal OR NOT TARGET ETL::StdFull)
    message(FATAL_ERROR "AVR profile smoke builds require ETL_BUILD_STD=ON.")
endif()
if (NOT CMAKE_OBJCOPY)
    message(FATAL_ERROR "AVR profile smoke builds require avr-objcopy.")
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
# Compile exception/RTTI expressions without linking an ABI runtime into the smoke executable.
add_library(etl_avr_std_full_options_smoke OBJECT ${PROJECT_SOURCE_DIR}/examples/avr/std_full_options_smoke.cpp)
target_link_libraries(etl_avr_std_full_options_smoke PRIVATE ETL::StdFull)
foreach(profile minimal full)
    if (profile STREQUAL "minimal")
        set(library ETL::StdMinimal)
    else()
        set(library ETL::StdFull)
    endif()
    set(target etl_avr_std_${profile}_smoke)
    add_executable(${target} ${PROJECT_SOURCE_DIR}/examples/avr/std_${profile}_smoke.cpp)
    target_link_libraries(${target} PRIVATE ${library})
    target_compile_definitions(${target} PRIVATE F_CPU=${ETL_AVR_F_CPU})
    target_compile_options(${target} PRIVATE -Os -ffunction-sections -fdata-sections ${ETL_LIBSTD_WARNING_OPTIONS})
    target_link_options(${target} PRIVATE -Wl,--gc-sections "-Wl,-Map,${PROJECT_BINARY_DIR}/${target}.map")
    set_target_properties(${target} PROPERTIES SUFFIX .elf)
    # The ALL check runs even when the executable is up to date, including after deleting an artifact.
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom $<TARGET_FILE:${target}> ${PROJECT_BINARY_DIR}/${target}.hex
        BYPRODUCTS ${PROJECT_BINARY_DIR}/${target}.hex ${PROJECT_BINARY_DIR}/${target}.map)
    add_custom_target(${target}_check ALL
        COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/check_avr_matrix.py
            --artifacts ${PROJECT_BINARY_DIR} --profile ${profile}
        DEPENDS ${target} VERBATIM)
endforeach()
add_dependencies(etl_avr_std_full_smoke_check etl_avr_std_full_options_smoke)
