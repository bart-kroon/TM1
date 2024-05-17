cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

create_tmiv_library(TARGET PtlCheckerLib SOURCES "src/PtlChecker.cpp" PUBLIC MivBitstreamLib)

create_catch2_unit_test(TARGET PtlCheckerTest SOURCES "test/PtlChecker_test.cpp" PRIVATE
                        PtlCheckerLib)
