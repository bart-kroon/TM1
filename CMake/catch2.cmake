cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

option(BUILD_CATCH2 "Build and use Catch2 for unit tests in TMIV" ON)

if(BUILD_CATCH2)
    include(FetchContent)
    fetchcontent_declare(CATCH2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG "v2.11.1"
    )

    fetchcontent_makeavailable(CATCH2)

    set(CATCH2_CMAKE_FOLDER ${CMAKE_BINARY_DIR}/_deps/catch2-src/contrib/)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} CATCH2_CMAKE_FOLDER)

    set(CATCH2_INCLUDE_FOLDER ${CMAKE_BINARY_DIR}/_deps/catch2-src/single_include/)
    include_directories(CATCH2_INCLUDE_FOLDER)
endif()
