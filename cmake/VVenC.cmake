cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

option(BUILD_VVenC "Build VVenC" ON)
option(BUILD_VVdeC "Build VVdeC" ON)

if(BUILD_VVenC OR BUILD_VVdeC)
    include(FetchContent)
endif()

# TODO(BK): Switch to find_package because the coupling with fetchcontent is too tight and thus fragile
set (TMIV_CMAKE_CXX_STANDARD ${CMAKE_CXX_STANDARD})
set(CMAKE_CXX_STANDARD 14)

if(BUILD_VVenC)
    if(NO_INTERNET)
        set(LOCAL_VVENC_DIR ${CMAKE_SOURCE_DIR}/../vvenc-0.3.1.0 CACHE PATH "Path to the local VVenC directory" )
        message(STATUS "Looking for a local copy of VVenC in ${LOCAL_VVENC_DIR}")
        fetchcontent_declare(VVENC URL ${LOCAL_VVENC_DIR})
    else()
        fetchcontent_declare(VVENC
            GIT_REPOSITORY https://github.com/fraunhoferhhi/vvenc
            GIT_TAG "v0.3.1.0"
            GIT_PROGRESS TRUE
        )
    endif()

    set(vvenc_ADD_SUBDIRECTORIES "source/Lib/apputils;source/App/vvencFFapp")
    fetchcontent_makeavailable(VVENC)
    
    if (NOT MSVC)
        target_compile_options(vvenc PUBLIC "-w")        
    endif()
    
    set_property(TARGET vvenc apputils vvencFFapp PROPERTY FOLDER "VVenC")
    target_compile_features(vvenc PUBLIC cxx_std_14)
    install(TARGETS vvencFFapp EXPORT TMIVTargets RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    
    FetchContent_GetProperties(VVENC)
    install(
        DIRECTORY "${vvenc_SOURCE_DIR}/cfg/"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/config/vvenc")    
    install(
        FILES
            "${vvenc_SOURCE_DIR}/README.md"
            "${vvenc_SOURCE_DIR}/LICENSE.txt"
            "${vvenc_SOURCE_DIR}/AUTHORS.md"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/doc/VVenC")
endif()

if(BUILD_VVdeC)
    set(HAVE_VVDEC ON)

    if(NO_INTERNET)
        set(LOCAL_VVDEC_DIR ${CMAKE_SOURCE_DIR}/../vvdec-1.0.1 CACHE PATH "Path to the local VVdeC directory" )
        message(STATUS "Looking for a local copy of VVdeC in ${LOCAL_VVDEC_DIR}")
         fetchcontent_declare(VVDEC URL ${LOCAL_VVDEC_DIR})
    else()
        fetchcontent_declare(VVDEC
            GIT_REPOSITORY https://github.com/fraunhoferhhi/vvdec
            GIT_TAG "v1.0.1"
            GIT_PROGRESS TRUE
        )
    endif()

    set(vvdec_ADD_SUBDIRECTORIES "source/App/vvdecapp")
    fetchcontent_makeavailable(VVDEC)

    if (NOT MSVC)
        target_compile_options(vvdec PUBLIC "-w")
    endif()

    set_property(TARGET vvdec vvdecapp PROPERTY FOLDER "VVenC")
    target_compile_features(vvdec PUBLIC cxx_std_17)
    install(TARGETS vvdecapp EXPORT TMIVTargets RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

set(CMAKE_CXX_STANDARD ${TMIV_CMAKE_CXX_STANDARD})
