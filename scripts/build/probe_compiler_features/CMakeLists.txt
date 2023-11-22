cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(probe_compiler_features LANGUAGES CXX)

add_executable(probe_compiler_features "probe_compiler_features.cpp")
target_compile_features(probe_compiler_features PRIVATE cxx_std_17)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_20 cxx_std_20)
list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_23 cxx_std_23)
list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_26 cxx_std_26)

if(${cxx_std_26} GREATER_EQUAL 0)
    target_compile_features(probe_compiler_features PRIVATE cxx_std_26)
    target_compile_definitions(probe_compiler_features PRIVATE cxx_standard=26)
elseif(${cxx_std_23} GREATER_EQUAL 0)
    target_compile_features(probe_compiler_features PRIVATE cxx_std_23)
    target_compile_definitions(probe_compiler_features PRIVATE cxx_standard=23)
elseif(${cxx_std_20} GREATER_EQUAL 0)
    target_compile_features(probe_compiler_features PRIVATE cxx_std_20)
    target_compile_definitions(probe_compiler_features PRIVATE cxx_standard=20)
else()
    target_compile_definitions(probe_compiler_features PRIVATE cxx_standard=17)
endif()

install(
    TARGETS probe_compiler_features
    EXPORT Targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
