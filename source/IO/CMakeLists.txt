cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

# Address this https://en.cppreference.com/w/cpp/filesystem note: Using this library may require
# additional compiler/linker options. GNU implementation prior to 9.1 requires linking with
# -lstdc++fs and LLVM implementation prior to LLVM 9.0 requires linking with -lc++fs
if(NOT MSVC
   AND NOT MSYS
   AND NOT MINGW)
    if(${CMAKE_CXX_COMPILER_ID} STREQUAL GNU AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 9.1)
        message(STATUS "TMIV: Adding stdc++fs for C++ filesystem")
        set(CppFilesystemLib stdc++fs)
    elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL Clang AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS
                                                       9.0)
        message(STATUS "TMIV: Adding c++fs for C++ filesystem")
        set(CppFilesystemLib c++fs)
    endif()
endif()

create_tmiv_library(
    TARGET
    IOLib
    SOURCES
    "src/IO.cpp"
    "src/load.cpp"
    "src/save.cpp"
    "src/DependencyInjector.cpp"
    "src/Filesystem.cpp"
    PUBLIC
    MivBitstreamLib
    ${CppFilesystemLib})

create_catch2_unit_test(
    TARGET
    IOTest
    SOURCES
    "test/IO_test.cpp"
    "test/load_test.cpp"
    "test/save_test.cpp"
    "test/FakeFilesystem.cpp"
    PRIVATE
    IOLib)
