cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

find_or_fetch(
    NAME Catch2
    VERSION "2.13.8"
    GIT_URL "https://github.com/catchorg/Catch2.git"
    GIT_REF "v2.13.8"
    LOCAL_DIR LOCAL_CATCH2_DIR
    REQUIRED
)

if(FIND_OR_FETCH_FOUND)
    include("${Catch2_DIR}/Catch.cmake")
elseif(FIND_OR_FETCH_FETCHED)
    FetchContent_GetProperties(catch2)

    if(NOT catch2_POPULATED)
        FetchContent_Populate(catch2)
        set(CATCH_BUILD_STATIC_LIBRARY ON CACHE INTERNAL "")
        add_subdirectory(${catch2_SOURCE_DIR} ${catch2_BINARY_DIR})
        include("${catch2_SOURCE_DIR}/contrib/Catch.cmake")
    endif()
endif()
