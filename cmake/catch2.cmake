cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

include(FetchContent)

if(NO_INTERNET)
    set(LOCAL_CATCH2_DIR ${CMAKE_SOURCE_DIR}/../Catch2-2.13.8 CACHE PATH "Path to the local Catch2 directory" )
    message(STATUS "Looking for a local copy of the Catch2 test framework in ${LOCAL_CATCH2_DIR}")
    fetchcontent_declare(catch2 URL ${LOCAL_CATCH2_DIR})
else()
    fetchcontent_declare(catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG "v2.13.8"
            GIT_PROGRESS TRUE
            )
endif()

FetchContent_GetProperties(catch2)

if(NOT catch2_POPULATED)
    FetchContent_Populate(catch2)
    set(CATCH_BUILD_STATIC_LIBRARY ON CACHE INTERNAL "")
    add_subdirectory(${catch2_SOURCE_DIR} ${catch2_BINARY_DIR})
    include("${catch2_SOURCE_DIR}/contrib/Catch.cmake")
endif()
