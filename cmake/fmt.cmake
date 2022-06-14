cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

find_or_fetch(
    NAME FMT
    VERSION "8.1.1"
    GIT_URL "https://github.com/fmtlib/fmt.git"
    GIT_REF "8.1.1"
    REQUIRED
)

if (FIND_OR_FETCH_FETCHED)
    fetchcontent_makeavailable(FMT)

    install(
        TARGETS
            fmt
        EXPORT TMIVTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
