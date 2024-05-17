cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

project(
    TMIV
    DESCRIPTION "Test model for MPEG immersive video (TMIV)"
    VERSION 19.0.0
    LANGUAGES C CXX)

if(NOT DEFINED CMAKE_CXX_EXTENSIONS)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(NOT DEFINED CMAKE_CXX_STANDARD)
    list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_20 have_cxx_std_20)
    list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_23 have_cxx_std_23)

    if(${have_cxx_std_23} GREATER_EQUAL 0)
        set(CMAKE_CXX_STANDARD 23)
    elseif(${have_cxx_std_20} GREATER_EQUAL 0)
        set(CMAKE_CXX_STANDARD 20)
    else()
        set(CMAKE_CXX_STANDARD 17)
    endif()
endif()

option(ENABLE_IPO
       "Enable interprocedural optimization (requires more resources) for all but debug builds" OFF)

if(ENABLE_IPO)
    include(CheckIPOSupported)
    check_ipo_supported()
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF)
endif()

set(TMIV_BUILD_APP_LIST
    ""
    CACHE STRING
          "Build only the specified semicolon-separated list of applications. Empty to build all.")
set(TMIV_BUILD_LIB_LIST
    ""
    CACHE STRING
          "Build only the specified semicolon-separated list of libraries. Empty to build all.")
set(TMIV_BUILD_TEST_LIST
    ""
    CACHE STRING "Build only the specified semicolon-separated list of tests. Empty to build all.")

# Advanced use: the user needs to take care of the dependencies
mark_as_advanced(TMIV_BUILD_APP_LIST)
mark_as_advanced(TMIV_BUILD_LIB_LIST)
mark_as_advanced(TMIV_BUILD_TEST_LIST)

include(CTest)
include(GNUInstallDirs)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

message(STATUS "TMIV: Building in C++${CMAKE_CXX_STANDARD} mode.")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
include(cmake/fmt.cmake)
include(cmake/tmiv_module_creation.cmake)
include(cmake/test_creation.cmake)

# For experimentation purposes, the test model may implement proposals that are not yet adopted. By
# default the aim of the test model is to encode and decode valid bitstreams. Compilation of these
# proposals is therefore controlled by a CMake option and precompiler flag. When later it would be
# decided to adopt (or reject) the proposal the option (and code) will be removed by the software
# coordinators.

# option(ENABLE_M12345 "Enable experimental support for MPEG/M12345" OFF)

foreach(
    subdir
    Common
    MivBitstream
    IO
    ViewOptimizer
    Pruner
    Aggregator
    Packer
    Quantizer
    Downscaler
    FramePacker
    Renderer
    ViewingSpace
    VideoDecoder
    Encoder
    Decoder
    Multiplexer
    Parser
    PtlChecker
    GaInserter
    EgaInserter
    DepthQualityAssessor
    MpiPcs
    BitstreamMerger)
    add_subdirectory("source/${subdir}")
endforeach()

if(NOT TARGET module_graph)
    add_custom_target(
        module_graph COMMAND ${CMAKE_COMMAND} "-DDOC_DIR=${CMAKE_SOURCE_DIR}/doc" "-P"
                             "${CMAKE_CURRENT_LIST_DIR}/cmake/module_graph.cmake")
    set_property(TARGET module_graph PROPERTY FOLDER "Utilities")
endif()

# To use TMIV as a sub project, clone, build and install. Then in the CMake GUI set TMIV_DIR to the
# lib/cmake/TMIV directory of the TMIV installation. In the super project CMakeLists.txt use
# find_package(TMIV REQUIRED)
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

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/TMIVConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/TMIVConfigVersion.cmake"
        DESTINATION ${TMIV_CMAKE_CONFIG_DESTINATION})

install(DIRECTORY "config" DESTINATION ${CMAKE_INSTALL_DATADIR})
install(FILES "README.md" "LICENSE" DESTINATION ${CMAKE_INSTALL_DOCDIR})
install(DIRECTORY "doc" DESTINATION ${CMAKE_INSTALL_DOCDIR})
