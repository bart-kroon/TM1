cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

include(FetchContent)
# TODO(BK): Restore NO_INTERNET capability, which requires making a new Docker image for CI
#
# if(NO_INTERNET)
#    set(LOCAL_FMT_DIR ${CMAKE_SOURCE_DIR}/../fmt-7.0.3 CACHE PATH "Path to the local fmt directory")
#    message(STATUS "Looking for a local copy of the {fmt} strong formatting library in ${LOCAL_FMT_DIR}")
#    fetchcontent_declare(FMT URL ${LOCAL_FMT_DIR})
# else()
    fetchcontent_declare(FMT
            GIT_REPOSITORY https://github.com/fmtlib/fmt.git
            GIT_TAG "8.1.1"
            GIT_PROGRESS TRUE
            )
# endif()

fetchcontent_makeavailable(FMT)
