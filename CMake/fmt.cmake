cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

include(FetchContent)
fetchcontent_declare(FMT
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG "7.0.3"
    GIT_PROGRESS TRUE
)
fetchcontent_makeavailable(FMT)
