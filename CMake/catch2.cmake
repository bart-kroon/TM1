cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

option(BUILD_CATCH2 "Build and use Catch2 for unit tests in TMIV" ON)

if(BUILD_CATCH2)
    SET(CATCH2_ROOT thirdparty/catch2)
    include(ExternalProject)
    externalproject_add(Catch2 
        PREFIX ${CATCH2_ROOT}
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG "v2.11.1"
        INSTALL_DIR ${CATCH2_ROOT}/install
        INSTALL_COMMAND make DESTDIR=<INSTALL_DIR> install
    )
    SET(Catch2_DIR ${CMAKE_BINARY_DIR}/${CATCH2_ROOT}/install/usr/local/lib/cmake/Catch2)

endif()
