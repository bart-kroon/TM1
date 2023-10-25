cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
    DownscalerLib
    SOURCES
    "src/GeometryDownscaler.cpp"
    "src/Stage.cpp"
    PUBLIC
    MivBitstreamLib)
