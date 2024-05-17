cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

create_tmiv_library(
    TARGET
    DownscalerLib
    SOURCES
    "src/GeometryDownscaler.cpp"
    "src/DownscalerStage.cpp"
    PUBLIC
    MivBitstreamLib)
