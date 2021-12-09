cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

if (HAVE_HM)
    set(hmSources
        "src/HmVideoDecoder.cpp"
    )
endif()

if (HAVE_VVDEC)
    set(vvdecSources
        "src/VVdeCVideoDecoder.cpp")
endif()

create_tmiv_library(
    TARGET
        VideoDecoderLib
    SOURCES
        "src/VideoDecoderBase.cpp"
        "src/VideoDecoderFactory.cpp"
        ${hmSources}
        ${vvdecSources}
    PUBLIC
        TMIV::CommonLib
    )

if (HAVE_HM)
    target_link_libraries(VideoDecoderLib PRIVATE TMIV::TLibDecoder)
    target_compile_definitions(VideoDecoderLib PUBLIC HAVE_HM)
endif()

if (HAVE_VVDEC)
    target_link_libraries(VideoDecoderLib PRIVATE vvdec)
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
        "src/VideoDecoderBase.test.cpp"
        "src/VideoDecoderFactory.test.cpp"
    PRIVATE
        TMIV::VideoDecoderLib
    )
