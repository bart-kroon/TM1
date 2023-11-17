cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(probe_compiler_features LANGUAGES CXX)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_17 have_cxx_std_17)
list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_20 have_cxx_std_20)
list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_23 have_cxx_std_23)

if(${have_cxx_std_23} GREATER_EQUAL 0)
    set(HAVE_CXX_STANDARD 23)
elseif(${have_cxx_std_20} GREATER_EQUAL 0)
    set(HAVE_CXX_STANDARD 20)
elseif(${have_cxx_std_17} GREATER_EQUAL 0)
    set(HAVE_CXX_STANDARD 17)
else()
    message(FATAL_ERROR "TMIV requires at least a C++17 compiler")
endif()

set(HAVE_CXX_STANDARD
    ${HAVE_CXX_STANDARD}
    CACHE STRING "Available C++ standard (17, 20 or 23)" FORCE)
