cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

option(USE_CLANG_TIDY "Turn on clang_tidy processing if available" ON)

if (USE_CLANG_TIDY)
    find_program(CLANG_TIDY NAMES "clang-tidy")

    if(CLANG_TIDY)
        set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY}")
    endif()
endif()

