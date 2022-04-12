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
        RendererLib
    )

create_catch2_unit_test(
    TARGET
        ViewOptimizerTest
    SOURCES
        "src/ServerSideInpainter.test.cpp"
    PRIVATE
        ViewOptimizerLib
    )
