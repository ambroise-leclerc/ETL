include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CheckCXXCompilerFlag)

if (NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "avr" OR NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "ETLStd requires AVR GCC; configure with cmake/avr-gcc-toolchain.cmake.")
endif()
if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 14.2)
    message(FATAL_ERROR "ETLStd requires AVR GCC 14.2 or newer for C++23 and -nostdlib++.")
endif()
if (NOT DEFINED ETL_AVR_MCU OR ETL_AVR_MCU STREQUAL "")
    message(FATAL_ERROR "ETLStd requires a nonempty ETL_AVR_MCU.")
endif()
execute_process(COMMAND "${CMAKE_CXX_COMPILER}" "-mmcu=${ETL_AVR_MCU}" -print-multi-directory
                RESULT_VARIABLE result OUTPUT_VARIABLE ETLSTD_MULTILIB OUTPUT_STRIP_TRAILING_WHITESPACE)
if (NOT result EQUAL 0 OR ETLSTD_MULTILIB STREQUAL "" OR ETLSTD_MULTILIB MATCHES "(^/|\\.\\.)")
    message(FATAL_ERROR "Cannot resolve AVR GCC multilib for ETL_AVR_MCU=${ETL_AVR_MCU}.")
endif()
foreach(flag -ffreestanding -fno-exceptions -fno-rtti -fexceptions -frtti -nostdinc++ -nostdlib++)
    string(MAKE_C_IDENTIFIER "ETLSTD_SUPPORTS_${flag}" check)
    check_cxx_compiler_flag("${flag}" ${check})
    if (NOT ${check})
        message(FATAL_ERROR "ETLStd requires compiler support for ${flag}.")
    endif()
endforeach()

add_library(etlstd_headers INTERFACE)
set_target_properties(etlstd_headers PROPERTIES EXPORT_NAME StdHeaders)
target_include_directories(etlstd_headers SYSTEM INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/libstd/include/c++/v1>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ETLStd/c++/v1>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ETLStd/vendor>)

foreach(ETL_PROFILE minimal full)
    if (ETL_PROFILE STREQUAL "minimal")
        set(export_name StdMinimal)
        set(ETL_PROFILE_EXCEPTIONS 0)
        set(profile_flags -fno-exceptions -fno-rtti)
    else()
        set(export_name StdFull)
        set(ETL_PROFILE_EXCEPTIONS 1)
        set(profile_flags -fexceptions -frtti)
    endif()
    set(config_dir "${PROJECT_BINARY_DIR}/etlstd/${ETL_PROFILE}")
    configure_file(${CMAKE_CURRENT_LIST_DIR}/etlstd/config.in ${config_dir}/__etl/config @ONLY)
    configure_file(${PROJECT_SOURCE_DIR}/libstd/runtime/profile.cpp.in ${config_dir}/profile.cpp @ONLY)
    add_library(etlstd_${ETL_PROFILE} STATIC ${config_dir}/profile.cpp)
    add_library(ETL::${export_name} ALIAS etlstd_${ETL_PROFILE})
    set_target_properties(etlstd_${ETL_PROFILE} PROPERTIES
        EXPORT_NAME ${export_name} OUTPUT_NAME etlstd_${ETL_PROFILE})
    target_link_libraries(etlstd_${ETL_PROFILE} PUBLIC etlstd_headers)
    target_include_directories(etlstd_${ETL_PROFILE} SYSTEM BEFORE PUBLIC
        $<BUILD_INTERFACE:${config_dir}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ETLStd/${ETL_PROFILE}>)
    target_compile_options(etlstd_${ETL_PROFILE} PUBLIC
        -std=c++23 -ffreestanding ${profile_flags} -nostdinc++
        "SHELL:-include __etl/config")
    target_link_options(etlstd_${ETL_PROFILE} INTERFACE -nostdlib++)
    install(TARGETS etlstd_${ETL_PROFILE} EXPORT ETLStdTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/ETLStd/${ETLSTD_MULTILIB} COMPONENT ETLStd)
    install(FILES ${config_dir}/__etl/config
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ETLStd/${ETL_PROFILE}/__etl COMPONENT ETLStd)
endforeach()
install(TARGETS etlstd_headers EXPORT ETLStdTargets COMPONENT ETLStd)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/libstd/include/c++/v1/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ETLStd/c++/v1 COMPONENT ETLStd)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ETLStd/vendor COMPONENT ETLStd)
include(${CMAKE_CURRENT_LIST_DIR}/ETLStdValidate.cmake)
configure_file(${CMAKE_CURRENT_LIST_DIR}/ETLStdValidate.cmake
    ${PROJECT_BINARY_DIR}/ETLStdValidate.cmake COPYONLY)
install(FILES ${PROJECT_BINARY_DIR}/ETLStdValidate.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ETLStd COMPONENT ETLStd)
set(package_dir ${CMAKE_INSTALL_LIBDIR}/cmake/ETLStd)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/ETLStdConfig.cmake.in
    ${PROJECT_BINARY_DIR}/ETLStdConfig.cmake INSTALL_DESTINATION ${package_dir})
write_basic_package_version_file(${PROJECT_BINARY_DIR}/ETLStdConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
export(TARGETS etlstd_headers etlstd_minimal etlstd_full
    NAMESPACE ETL:: FILE ${PROJECT_BINARY_DIR}/ETLStdTargets.cmake)
install(EXPORT ETLStdTargets NAMESPACE ETL:: DESTINATION ${package_dir} COMPONENT ETLStd)
install(FILES ${PROJECT_BINARY_DIR}/ETLStdConfig.cmake ${PROJECT_BINARY_DIR}/ETLStdConfigVersion.cmake
    DESTINATION ${package_dir} COMPONENT ETLStd)

include(CTest)
if (BUILD_TESTING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME etlstd-profile-package
        COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/test_etlstd_profiles.py
            --build ${PROJECT_BINARY_DIR} --compiler ${CMAKE_CXX_COMPILER} --mcu ${ETL_AVR_MCU})
endif()
