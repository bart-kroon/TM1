function(create_catch2_unit_test)
    if(NOT CMAKE_TESTING_ENABLED)
        return()
    endif()

    set(prefix TEST_CREATOR)
    set(flags)
    set(singleValues TARGET)
    set(multiValues SOURCES PRIVATE)

    include(CMakeParseArguments)
    cmake_parse_arguments(${prefix}
            "${flags}"
            "${singleValues}"
            "${multiValues}"
            ${ARGN})

    if (NOT "${TMIV_BUILD_TEST_LIST}" STREQUAL "")
        set(found_test OFF)

        foreach (enabled_test ${TMIV_BUILD_TEST_LIST})
            if (${enabled_test} STREQUAL ${TEST_CREATOR_TARGET})
                set(found_test ON)
                break()
            endif()
        endforeach()

        if (NOT found_test)
            message(STATUS "Test ${TEST_CREATOR_TARGET} is excluded from the build.")
            return()
        endif()
    endif ()
    
    if(NOT TARGET ${TEST_CREATOR_TARGET})
        add_executable(${TEST_CREATOR_TARGET} ${TEST_CREATOR_SOURCES})

        target_link_libraries(${TEST_CREATOR_TARGET}
            PRIVATE
                Catch2::Catch2WithMain
                ${TEST_CREATOR_PRIVATE})

        if(CLANG_TIDY_PATH)
            set_target_properties(${TEST_CREATOR_TARGET}
                    PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_PATH};-checks=-readability-function-size,-readability-magic-numbers,-readability-function-cognitive-complexity")
        endif()

        set_property(TARGET ${TEST_CREATOR_TARGET} PROPERTY FOLDER "TMIV tests")

        catch_discover_tests(${TEST_CREATOR_TARGET})
    endif()
endfunction()
