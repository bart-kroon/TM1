cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

if (HAVE_HM)
    set(hmSources
        "src/HmVideoDecoder.cpp"
    )
endif()

create_tmiv_library(
    TARGET
        VideoDecoderLib
    SOURCES
        "src/VideoDecoderFactory.cpp"
        "src/VideoDecoderBase.cpp"
        ${hmSources}
    PUBLIC
        TMIV::MivBitstreamLib
    )

if (HAVE_HM)
    target_link_libraries(VideoDecoderLib PRIVATE TMIV::TLibDecoder)
    target_compile_definitions(VideoDecoderLib PRIVATE HAVE_HM)
endif()
