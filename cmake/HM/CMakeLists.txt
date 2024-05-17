cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

project(
    HM
    DESCRIPTION "HEVC test model (HM) with CMake project provided by TMIV"
    VERSION 16.16
    LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_STANDARD 14)
endif()

include(GNUInstallDirs)

function(add_hm_executable module)
    file(GLOB cppSourceFiles "${CMAKE_CURRENT_SOURCE_DIR}/source/App/${module}/*.cpp")
    file(GLOB cSourceFiles "${CMAKE_CURRENT_SOURCE_DIR}/source/App/${module}/*.c")
    file(GLOB headerFiles "${CMAKE_CURRENT_SOURCE_DIR}/source/App/${module}/*.h")
    add_executable(${module} ${cppSourceFiles} ${cSourceFiles} ${headerFiles})
    install(
        TARGETS ${module}
        EXPORT HMTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endfunction()

function(add_hm_library module)
    file(GLOB cppSourceFiles "${CMAKE_CURRENT_SOURCE_DIR}/source/Lib/${module}/*.cpp")
    file(GLOB cSourceFiles "${CMAKE_CURRENT_SOURCE_DIR}/source/Lib/${module}/*.c")
    file(GLOB headerFiles "${CMAKE_CURRENT_SOURCE_DIR}/source/Lib/${module}/*.h")
    add_library(${module} ${cppSourceFiles} ${cSourceFiles})
    target_include_directories(
        ${module} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/source/Lib>"
                         "$<INSTALL_INTERFACE:include>")
    install(
        TARGETS ${module}
        EXPORT HMTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(FILES ${headerFiles} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${module})
endfunction()

add_hm_library(libmd5)
target_compile_features(libmd5 PUBLIC cxx_std_11)

add_hm_library(TLibCommon)
target_link_libraries(TLibCommon PRIVATE libmd5)
target_compile_features(TLibCommon PUBLIC cxx_std_11)
target_compile_definitions(TLibCommon PUBLIC "$<$<CXX_COMPILER_ID:MSVC>:_CRT_SECURE_NO_WARNINGS>")
target_compile_options(TLibCommon PUBLIC "$<$<CXX_COMPILER_ID:Clang>:-w>")
target_compile_options(TLibCommon PUBLIC "$<$<CXX_COMPILER_ID:GNU>:-w>")
target_compile_options(
    TLibCommon
    PUBLIC
        "$<$<CXX_COMPILER_ID:MSVC>:/wd4100;/wd4189;/wd4127;/wd4244;/wd4389;/wd4459;/wd4701;/wd4702>"
)

add_hm_library(TLibDecoder)
target_link_libraries(TLibDecoder PUBLIC TLibCommon)

add_hm_library(TAppCommon)
target_link_libraries(TAppCommon PUBLIC TLibCommon)

add_hm_library(TLibVideoIO)
target_link_libraries(TLibVideoIO PUBLIC TLibCommon)

add_hm_executable(TAppDecoder)
target_link_libraries(TAppDecoder PRIVATE TLibDecoder TAppCommon TLibVideoIO)

add_hm_library(TLibEncoder)
target_link_libraries(TLibEncoder PUBLIC TLibCommon)
target_compile_options(TLibEncoder PRIVATE "$<$<CXX_COMPILER_ID:MSVC>:/wd4267>")

add_hm_executable(TAppEncoder)
target_link_libraries(TAppEncoder PRIVATE TLibEncoder TAppCommon TLibVideoIO)

install(DIRECTORY "cfg/" DESTINATION "${CMAKE_INSTALL_DATADIR}/config/hm")
install(FILES "README" "COPYING" "doc/software-manual.pdf"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/doc/HM")

install(
    EXPORT HMTargets
    FILE "HMTargets.cmake"
    NAMESPACE HM::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/HM")
export(EXPORT HMTargets NAMESPACE HM::)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_LIST_DIR}/HMConfig.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/HMConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/HM")

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/HMConfigVersion.cmake"
    VERSION ${HM_VERSION}
    COMPATIBILITY SameMajorVersion)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/HMConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/HMConfigVersion.cmake"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/HM")
