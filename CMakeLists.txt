cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(
    TMIV
    DESCRIPTION "Test Model for Immersive Video"
    VERSION 7.0.0
    LANGUAGES C CXX)

set(CMAKE_CXX_EXTENSIONS OFF)

include(CMake/clang_tidy.cmake)

include(CMake/downloadcatch2.cmake)

# find_package(Catch2 2.10.2 QUIET)
if (BUILD_CATCH2)
    # add_subdirectory(${Catch2_DIR})
    add_subdirectory(${CMAKE_BINARY_DIR}/catch/src/catch_src)
    include_directories(${CATCH_INCLUDE_DIR} ${COMMON_INCLUDES})
    enable_testing(true)  # Enables unit-testing.

    # enable_testing()
endif()

include(GNUInstallDirs)

include(CMake/HM.cmake)

foreach(subdir
        Common 
        MivBitstream 
        IO
        ViewOptimizer
        Pruner
        Aggregator
        Packer
        GeometryQuantizer
        Renderer
        ViewingSpace        
        VideoDecoder
        Encoder
        Decoder
        Parser
        DepthQualityAssessor)
    add_subdirectory(${subdir})
endforeach()

add_custom_target(clang_format
    COMMAND
        ${CMAKE_COMMAND}
        "-DANALYZE_DIR=${CMAKE_SOURCE_DIR}"
        "-P"
        "${CMAKE_CURRENT_LIST_DIR}/CMake/clang_format.cmake")

# To use TMIV as a sub project, clone, build and install.
# Then in the CMake GUI set TMIV_DIR to the lib/cmake/TMIV directory of the TMIV installation.
# In the super project CMakeLists.txt use find_package(TMIV 7.0.0 REQUIRED)
set(TMIV_CMAKE_CONFIG_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/TMIV")

install(
    EXPORT TMIVTargets
    FILE "TMIVTargets.cmake"
    NAMESPACE TMIV::
    DESTINATION ${TMIV_CMAKE_CONFIG_DESTINATION})
export(EXPORT TMIVTargets NAMESPACE TMIV::)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_LIST_DIR}/CMake/TMIVConfig.cmake.in"
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
