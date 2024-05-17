cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

create_tmiv_library(TARGET AggregatorLib SOURCES "src/Aggregator.cpp" PUBLIC CommonLib)

create_catch2_unit_test(TARGET AggregatorTest SOURCES "test/Aggregator_test.cpp" PRIVATE
                        AggregatorLib)
