cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

find_program(DOT_PATH NAMES "dot")

# Small graph for the viewers of README.md (e.g. learning about the project)

file(
    WRITE "${CMAKE_BINARY_DIR}/CMakeGraphVizOptions.cmake"
    "set(GRAPHVIZ_IGNORE_TARGETS \"Test|Catch2|fmt|Threads|fmt.header.only|apputils|vvdec|vvenc|TApp|TLib|libmd5\")\n"
    "set(GRAPHVIZ_GENERATE_PER_TARGET FALSE)\n"
    "set(GRAPHVIZ_GENERATE_DEPENDERS FALSE)\n")

execute_process(COMMAND ${CMAKE_COMMAND} "--graphviz=module_graph" .
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

if(DOT_PATH)
    set(OUTPUT_FILE "${DOC_DIR}/module_graph.svg")
    execute_process(COMMAND ${DOT_PATH} "module_graph" "-Tsvg" "-o" "${OUTPUT_FILE}"
                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
    message(STATUS "TMIV: The module graph has been generated and rendered to ${OUTPUT_FILE}")
else()
    message(
        WARNING
            "TMIV: The module graph has been generated but Graphviz is required to render the .dot-file"
    )
endif()

# Full module graph for expert use (e.g. checking module dependencies)

file(WRITE "${CMAKE_BINARY_DIR}/CMakeGraphVizOptions.cmake"
     "set(GRAPHVIZ_GENERATE_PER_TARGET FALSE)\n" "set(GRAPHVIZ_GENERATE_DEPENDERS FALSE)\n")

execute_process(COMMAND ${CMAKE_COMMAND} "--graphviz=module_graph" .
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

if(DOT_PATH)
    set(OUTPUT_FILE "${DOC_DIR}/detailed_module_graph.svg")
    execute_process(COMMAND ${DOT_PATH} "module_graph" "-Tsvg" "-o" "${OUTPUT_FILE}"
                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
    message(STATUS "TMIV: The module graph has been generated and rendered to ${OUTPUT_FILE}")
else()
    message(
        WARNING
            "TMIV: The module graph has been generated but Graphviz is required to render the .dot-file"
    )
endif()
