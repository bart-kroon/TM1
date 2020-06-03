cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(GeometryQuantizerLib
    "src/GeometryQuantizer.cpp"
    "src/ExplicitOccuapncy.cpp"
    "include/TMIV/GeometryQuantizer/ExplicitOccuapncy.h"
    "include/TMIV/GeometryQuantizer/IGeometryQuantizer.h"
    "include/TMIV/GeometryQuantizer/GeometryQuantizer.h"
    )

add_library(TMIV::GeometryQuantizerLib
    ALIAS GeometryQuantizerLib)

target_link_libraries(GeometryQuantizerLib
    PUBLIC
        TMIV::MivBitstreamLib)

target_include_directories(GeometryQuantizerLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
    TARGETS
        GeometryQuantizerLib
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if (CMAKE_TESTING_ENABLED)
    add_executable(GeometryQuantizerTest "src/GeometryQuantizer.test.cpp")

    target_link_libraries(GeometryQuantizerTest
        PRIVATE
            Catch2::Catch2
            TMIV::GeometryQuantizerLib)

    catch_discover_tests(GeometryQuantizerTest)
endif()    
