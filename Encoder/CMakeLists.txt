cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

add_library(EncoderLib
    "src/Encoder.cpp"
    "src/Encoder_prepareSequence.cpp"
    "src/Encoder_prepareAccessUnit.cpp"
    "src/Encoder_pushFrame.cpp"
    "src/Encoder_completeAccessUnit.cpp"
    "src/Encoder_popFrame.cpp"
    "src/GroupBasedEncoder.cpp"
    "src/GeometryDownscaler.cpp"
    "src/MivEncoder.cpp"
    "include/TMIV/Encoder/IEncoder.h"
    "include/TMIV/Encoder/Encoder.h"
    "include/TMIV/Encoder/GroupBasedEncoder.h"
    "include/TMIV/Encoder/GeometryDownscaler.h"
    "include/TMIV/Encoder/MivEncoder.h"
    )

add_library(TMIV::EncoderLib
    ALIAS EncoderLib)

target_link_libraries(EncoderLib
    PUBLIC
        TMIV::ViewOptimizerLib
        TMIV::PrunerLib
        TMIV::AggregatorLib
        TMIV::PackerLib
        TMIV::GeometryQuantizerLib
        TMIV::DepthQualityAssessorLib)

target_include_directories(EncoderLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

add_executable(Encoder
    "src/Encoder.main.cpp"
    "src/registerComponents.cpp"
    "src/IvMetadataWriter.cpp"
    "src/IvMetadataWriter.h"
    )

target_link_libraries(Encoder
    PRIVATE
        TMIV::EncoderLib
        TMIV::IOLib)

add_executable(Multiplexer
    "src/Multiplexer.main.cpp"
    )

target_link_libraries(Multiplexer
    PRIVATE
        TMIV::MivBitstreamLib
        TMIV::IOLib
        stdc++fs)

install(
    TARGETS
        Encoder
        EncoderLib
        Multiplexer
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
    DIRECTORY "include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
