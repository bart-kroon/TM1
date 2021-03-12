cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

# Address this https://en.cppreference.com/w/cpp/filesystem note:
#   Using this library may require additional compiler/linker options. GNU implementation prior to
#   9.1 requires linking with -lstdc++fs and LLVM implementation prior to LLVM 9.0 requires linking
#   with -lc++fs
if(MSVC OR MSYS OR MINGW)
    message(STATUS "C++ filesystem does not require a library on this platform")
else()
    if (${CMAKE_CXX_COMPILER_ID} STREQUAL GNU AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 9.1)
        message(STATUS "Adding stdc++fs for C++ filesystem")
        set(CppFilesystemLib stdc++fs)
    elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL Clang AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 9.0)
        message(STATUS "Adding c++fs for C++ filesystem")
        set(CppFilesystemLib c++fs)
    endif()
endif()

create_tmiv_library(
    TARGET
        MpiPcsLib
    SOURCES
        "src/MpiPcs.cpp"
    PUBLIC
        TMIV::IOLib
        ${CppFilesystemLib}
    )

create_tmiv_executable(
    TARGET
        MpiPcs
    SOURCES
        "src/MpiPcs.main.cpp"
    PRIVATE
        TMIV::MpiPcsLib
    )

create_catch2_unit_test(
    TARGET
        MpiPcsTest
    SOURCES
        "src/MpiPcs.test.cpp"
    PRIVATE
        TMIV::MpiPcsLib
    )