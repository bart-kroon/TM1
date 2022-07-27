cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        ViewOptimizerLib
    SOURCES
        "src/AbstractViewSelector.cpp"
        "src/BasicViewAllocator.cpp"
        "src/KMedoidsCost.cpp"
        "src/ServerSideInpainter.cpp"
    PUBLIC
        MivBitstreamLib
    PRIVATE
        RendererLib
        fmt::fmt
    )

create_catch2_unit_test(
    TARGET
        ViewOptimizerTest
    SOURCES
        "src/BasicViewAllocator.test.cpp"
        "src/ServerSideInpainter.test.cpp"
    PRIVATE
        ViewOptimizerLib
        RendererLib
        fmt::fmt
    )
