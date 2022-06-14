cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

find_or_fetch(
    NAME vvenc
    VERSION "0.3.1.0"
    GIT_URL "https://github.com/fraunhoferhhi/vvenc"
    GIT_REF "v0.3.1.0"
    BUILD_FLAG BUILD_VVenC
)

if(FIND_OR_FETCH_FETCHED)
    set(TMIV_CMAKE_CXX_STANDARD ${CMAKE_CXX_STANDARD})
    set(CMAKE_CXX_STANDARD 14)
    set(vvenc_ADD_SUBDIRECTORIES "source/Lib/apputils;source/App/vvencFFapp")
    fetchcontent_makeavailable(vvenc)
    set(CMAKE_CXX_STANDARD ${TMIV_CMAKE_CXX_STANDARD})
    
    if (NOT MSVC)
        target_compile_options(vvenc PUBLIC "-w")        
    endif()
    
    set_property(TARGET vvenc apputils vvencFFapp PROPERTY FOLDER "VVenC")
    target_compile_features(vvenc PUBLIC cxx_std_14)
    install(TARGETS vvencFFapp EXPORT TMIVTargets RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    
    FetchContent_GetProperties(vvenc)
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
