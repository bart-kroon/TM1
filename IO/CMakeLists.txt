cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

add_library(IOLib
    "src/IO.cpp"
    "include/TMIV/IO/IO.h"
    "include/TMIV/IO/IO.hpp"
    )

add_library(TMIV::IOLib
    ALIAS IOLib)

# Address this https://en.cppreference.com/w/cpp/filesystem note:
#   Using this library may require additional compiler/linker options. GNU implementation prior to
#   9.1 requires linking with -lstdc++fs and LLVM implementation prior to LLVM 9.0 requires linking
#   with -lc++fs
if (${CMAKE_CXX_COMPILER_ID} STREQUAL GNU AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 9.1)
    set(CppFilesystemLib stdc++fs)
elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL Clang AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 9.0)
    if(NOT(MSVC OR MSYS OR MINGW))  # don't include c++fs for clang-cl
        set(CppFilesystemLib c++fs)
    endif()
endif()

target_link_libraries(IOLib
    PUBLIC
        TMIV::DecoderLib ${CppFilesystemLib})

target_include_directories(IOLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
    TARGETS
        IOLib
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
