cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
    PrunerLib
    SOURCES
    "src/computeOverlappingMatrix.cpp"
    "src/HierarchicalPruner.cpp"
    "src/LumaStdDev.cpp"
    "src/NoPruner.cpp"
    "src/PrunedMesh.cpp"
    PRIVATE
    RendererLib
    ${TMIV_FMT_TARGET})

create_catch2_unit_test(
    TARGET
    PrunerTest
    SOURCES
    "test/LumaStdDev_test.cpp"
    "test/HierarchicalPruner_test.cpp"
    PRIVATE
    PrunerLib
    RendererLib)
