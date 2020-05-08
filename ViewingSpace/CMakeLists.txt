cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(ViewingSpaceLib
    "src/SignedDistance.cpp"
    "src/ViewingSpaceEvaluator.cpp"
    "include/TMIV/ViewingSpace/SignedDistance.h"
    "include/TMIV/ViewingSpace/ViewingSpaceEvaluator.h"
    )

add_library(TMIV::ViewingSpaceLib
    ALIAS ViewingSpaceLib)

target_link_libraries(ViewingSpaceLib
    PUBLIC
        TMIV::MivBitstreamLib)

target_include_directories(ViewingSpaceLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
    TARGETS
        ViewingSpaceLib
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    
if (CMAKE_TESTING_ENABLED)
    add_executable(ViewingSpaceTest "src/ViewingSpace.test.cpp")

    target_link_libraries(ViewingSpaceTest
        PRIVATE
            Catch2::Catch2
            TMIV::ViewingSpaceLib)

    catch_discover_tests(ViewingSpaceTest)
endif()
