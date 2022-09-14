cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(
    TMIV
    DESCRIPTION "Test model for MPEG immersive video (TMIV)"
    VERSION 14.1.0
    LANGUAGES C CXX)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_STANDARD 17)
endif()

set(TMIV_BUILD_APP_LIST "" CACHE STRING "Build only the specified semicolon-separated list of applications. Empty to build all.")
set(TMIV_BUILD_LIB_LIST "" CACHE STRING "Build only the specified semicolon-separated list of libraries. Empty to build all.")
set(TMIV_BUILD_TEST_LIST "" CACHE STRING "Build only the specified semicolon-separated list of tests. Empty to build all.")

# Advanced use: the user needs to take care of the dependencies
mark_as_advanced(TMIV_BUILD_APP_LIST)
mark_as_advanced(TMIV_BUILD_LIB_LIST)
mark_as_advanced(TMIV_BUILD_TEST_LIST)

option(NO_INTERNET "Use pre-downloaded source archives for external libraries, e.g. Catch2, HM, and fmt" OFF)
set(ignore_when_not_used "${NO_INTERNET}") # Do not issue a warning when this flag is set and all dependencies are found locally

include(cmake/find_or_fetch.cmake)
include(cmake/fmt.cmake)

include(CTest)

if (BUILD_TESTING)
    include(cmake/catch2.cmake)
endif()

include(GNUInstallDirs)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
include(cmake/tmiv_module_creation.cmake)
include(cmake/test_creation.cmake)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include(cmake/HM.cmake)
include(cmake/VVenC.cmake)
include(cmake/VVdeC.cmake)

# Optionally enable clang-tidy, but only for TMIV modules
include(cmake/clang_tidy.cmake)

# For experimentation purposes, the test model may implement proposals that are not yet adopted. By
# default the aim of the test model is to encode and decode valid bitstreams. Compilation of these
# proposals is therefore controlled by a CMake option and precompiler flag. When later it would be
# decided to adopt (or reject) the proposal the option (and code) will be removed by the software
# coordinators.

option(ENABLE_M57419 "Enable experimental support for piecewise linear scaling [MPEG/M57419]" OFF)

foreach(subdir
        Common
        MivBitstream
        IO
        ViewOptimizer
        Pruner
        Aggregator
        Packer
        Renderer
        ViewingSpace
        VideoDecoder
        Encoder
        Decoder
        Multiplexer
        Parser
        PtlChecker
        GaInserter
        DepthQualityAssessor
        MpiPcs)
    add_subdirectory(source/${subdir})
endforeach()

if(NOT TARGET module_graph)
    add_custom_target(module_graph
        COMMAND ${CMAKE_COMMAND}
            "-DDOC_DIR=${CMAKE_SOURCE_DIR}/doc"
            "-P" "${CMAKE_CURRENT_LIST_DIR}/cmake/module_graph.cmake")
    set_property(TARGET module_graph PROPERTY FOLDER "Utilities")
endif()

# To use TMIV as a sub project, clone, build and install.
# Then in the CMake GUI set TMIV_DIR to the lib/cmake/TMIV directory of the TMIV installation.
# In the super project CMakeLists.txt use find_package(TMIV 14.1.0 REQUIRED)
set(TMIV_CMAKE_CONFIG_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/TMIV")

install(
    EXPORT TMIVTargets
    FILE "TMIVTargets.cmake"
    NAMESPACE TMIV::
    DESTINATION ${TMIV_CMAKE_CONFIG_DESTINATION})
export(EXPORT TMIVTargets NAMESPACE TMIV::)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_LIST_DIR}/cmake/TMIVConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/TMIVConfig.cmake"
    INSTALL_DESTINATION ${TMIV_CMAKE_CONFIG_DESTINATION})

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/TMIVConfigVersion.cmake"
    VERSION ${TMIV_VERSION}
    COMPATIBILITY SameMajorVersion)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/TMIVConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/TMIVConfigVersion.cmake"
    DESTINATION ${TMIV_CMAKE_CONFIG_DESTINATION})

install(
    DIRECTORY "config"
    DESTINATION ${CMAKE_INSTALL_DATADIR})
install(
    FILES
        "README.md"
        "LICENSE"
    DESTINATION ${CMAKE_INSTALL_DOCDIR})
install(
    DIRECTORY "doc"
    DESTINATION ${CMAKE_INSTALL_DOCDIR})
