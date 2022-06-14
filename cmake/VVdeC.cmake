cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

find_or_fetch(
    NAME vvdec
    VERSION "1.0.1"
    GIT_URL "https://github.com/fraunhoferhhi/vvdec"
    GIT_REF "v1.0.1"
    BUILD_FLAG BUILD_VVdeC
)

if(FIND_OR_FETCH_FETCHED)
    set(TMIV_CMAKE_CXX_STANDARD ${CMAKE_CXX_STANDARD})
    set(CMAKE_CXX_STANDARD 14)
    set(vvdec_ADD_SUBDIRECTORIES "source/App/vvdecapp")
    fetchcontent_makeavailable(vvdec)
    set(CMAKE_CXX_STANDARD ${TMIV_CMAKE_CXX_STANDARD})

    if (NOT MSVC)
        target_compile_options(vvdec PUBLIC "-w")
    endif()

    set_property(TARGET vvdec vvdecapp PROPERTY FOLDER "VVenC")
    target_compile_features(vvdec PUBLIC cxx_std_17)
    install(TARGETS vvdecapp EXPORT TMIVTargets RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(TARGETS vvdec EXPORT TMIVTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
    add_library(vvdec::vvdec ALIAS vvdec)
endif()

if(FIND_OR_FETCH_HAVE)
    set(HAVE_VVDEC ON)
endif()
