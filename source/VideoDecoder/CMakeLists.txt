cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

create_tmiv_library(
    TARGET
    VideoDecoderLib
    SOURCES
    "src/UnavailableVideoDecoders.cpp"
    "src/UnavailablePartitioners.cpp"
    PUBLIC
    CommonLib)

if(TARGET VideoDecoderLib)
    find_package(HM 16.16)

    if(HM_FOUND)
        message(STATUS "TMIV: Building on HM ${HM_VERSION}, found at ${HM_DIR}.")

        create_tmiv_library(
            TARGET
            VideoDecoderHmLib
            SOURCES
            "src/HM/HmPartition.cpp"
            "src/HM/HmVideoDecoder.cpp"
            PRIVATE
            CommonLib
            HM::TLibDecoder)
        set_property(TARGET VideoDecoderHmLib PROPERTY CXX_STANDARD 17) # HM headers are not valid
                                                                        # C++20
        target_link_libraries(VideoDecoderLib PRIVATE VideoDecoderHmLib)
        target_compile_definitions(VideoDecoderLib PUBLIC HAVE_HM)
    endif()

    find_package(vvdec 1)

    if(vvdec_FOUND)
        message(STATUS "TMIV: Building on VVdeC ${vvdec_VERSION}, found at ${vvdec_DIR}.")

        create_tmiv_library(
            TARGET
            VideoDecoderVvdecLib
            SOURCES
            "src/VVdeC/VVdeCPartition.cpp"
            "src/VVdeC/VVdeCVideoDecoder.cpp"
            PRIVATE
            CommonLib
            vvdec::vvdec)
        target_link_libraries(VideoDecoderLib PRIVATE VideoDecoderVvdecLib)
        target_compile_definitions(VideoDecoderLib PUBLIC HAVE_VVDEC)
    endif()

    configure_file(include/TMIV/VideoDecoder/Versions.h.in include/TMIV/VideoDecoder/Versions.h
                   @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/TMIV/VideoDecoder/Versions.h
            DESTINATION include/TMIV/VideoDecoder)

    target_include_directories(
        VideoDecoderLib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
                               "$<INSTALL_INTERFACE:include>")

    create_catch2_unit_test(
        TARGET
        VideoDecoderTest
        SOURCES
        "test/Partition_test.cpp"
        "test/VideoDecoder_test.cpp"
        PRIVATE
        VideoDecoderLib)
endif()
