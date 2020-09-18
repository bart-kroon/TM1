cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

option(ENABLE_CLANG_TIDY "Turn on clang_tidy processing if available" ON)

if (ENABLE_CLANG_TIDY)
    option(APPLY_CLANG_TIDY_FIXES "Let clang-tidy apply its auto fixes" ON)
    find_program(CLANG_TIDY_PATH NAMES "clang-tidy")

    if(CLANG_TIDY_PATH)
            set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_PATH}")
        if(APPLY_CLANG_TIDY_FIXES)
            string(APPEND CMAKE_CXX_CLANG_TIDY -fix)
        endif()
    endif()
endif()

