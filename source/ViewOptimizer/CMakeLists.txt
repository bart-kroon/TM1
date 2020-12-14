cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        ViewOptimizerLib
    SOURCES
        "src/AbstractViewSelector.cpp"
        "src/BasicViewAllocator.cpp"
        "src/KMedoidsCost.cpp"
        "include/TMIV/ViewOptimizer/IViewOptimizer.h"
        "include/TMIV/ViewOptimizer/AbstractViewSelector.h"
        "include/TMIV/ViewOptimizer/BasicViewAllocator.h"
        "include/TMIV/ViewOptimizer/KMedoidsCost.h"
    PUBLIC
        TMIV::RendererLib
    )
