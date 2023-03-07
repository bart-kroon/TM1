cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(TARGET AggregatorLib SOURCES "src/Aggregator.cpp" PUBLIC CommonLib)

create_catch2_unit_test(TARGET AggregatorTest SOURCES "src/Aggregator.test.cpp" PRIVATE
                        AggregatorLib)
