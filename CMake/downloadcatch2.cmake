cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

option(BUILD_CATCH2 "Build and use Catch2 for unit tests in TMIV" ON)

if(BUILD_CATCH2)
    # include(FetchContent)
    # fetchcontent_declare(CATCH2_REPO
    #     GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    #     # GIT_TAG "v2.11.1"
    # )

    # fetchcontent_makeavailable(CATCH2_REPO)

    # set(Catch2_DIR ${CMAKE_BINARY_DIR}/_deps/catch2_repo-src)

    # execute_process(
    #     COMMAND ${CMAKE_COMMAND} --build . ${CMAKE_BINARY_DIR}/${CATCH2_ROOT}/src/Catch2
    #     WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${CATCH2_ROOT}/src/Catch2-build)

    # execute_process(
    #     COMMAND ${CMAKE_COMMAND} --build . --target install
    #     WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${CATCH2_ROOT}/src/Catch2-build)

    # SET(Catch2_DIR ${CMAKE_BINARY_DIR}/${CATCH2_ROOT}/install/usr/local/lib/cmake/Catch2)
    include(ExternalProject)
    ExternalProject_Add(
        catch_src
        PREFIX ${CMAKE_BINARY_DIR}/catch
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        TIMEOUT 20
        UPDATE_COMMAND ${GIT_EXECUTABLE} pull
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
       )
    ExternalProject_Get_Property(catch_src source_dir)
    # add_library(catch INTERFACE)
    set(CATCH_INCLUDE_DIR ${source_dir}/include CACHE INTERNAL "Path to include folder for Catch")



endif()
