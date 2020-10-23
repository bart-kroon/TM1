cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

option(ENABLE_CLANG_TIDY "Turn on clang_tidy processing if available" ON)

if (ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_PATH NAMES "clang-tidy")

    if(CLANG_TIDY_PATH)
        set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_PATH}")
    else()
        message(WARNING "Couldn't find clang-tidy")
    endif()
endif()

