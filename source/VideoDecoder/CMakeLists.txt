cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        VideoDecoderLib
    SOURCES
        "src/UnavailableVideoDecoders.cpp"
        "src/UnavailablePartitioners.cpp"
    PUBLIC
        CommonLib
    PRIVATE
        fmt::fmt
    )

if (HAVE_HM AND TARGET VideoDecoderLib)
    create_tmiv_library(
        TARGET
            VideoDecoderHmLib
        SOURCES
            "src/HM/HmPartition.cpp"
            "src/HM/HmVideoDecoder.cpp"
        PRIVATE
            CommonLib
            TLibDecoder
        )
    set_property(TARGET VideoDecoderHmLib PROPERTY CXX_STANDARD 17) # HM headers are not valid C++20
    target_link_libraries(VideoDecoderLib PRIVATE VideoDecoderHmLib)
    target_compile_definitions(VideoDecoderLib PUBLIC HAVE_HM)
endif()

if (HAVE_VVDEC AND TARGET VideoDecoderLib)
    create_tmiv_library(
        TARGET
            VideoDecoderVvdecLib
        SOURCES
            "src/VVdeC/VVdeCPartition.cpp"
            "src/VVdeC/VVdeCVideoDecoder.cpp"
        PRIVATE
            CommonLib
            vvdec
            fmt::fmt
        )
    target_link_libraries(VideoDecoderLib PRIVATE VideoDecoderVvdecLib)
    target_compile_definitions(VideoDecoderLib PUBLIC HAVE_VVDEC)

    install(
        TARGETS
            vvdec
        EXPORT TMIVTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()


create_catch2_unit_test(
    TARGET
        VideoDecoderTest
    SOURCES
        "src/Partition.test.cpp"
        "src/VideoDecoder.test.cpp"
    PRIVATE
        VideoDecoderLib
        fmt::fmt
    )
