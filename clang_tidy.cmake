option(CLANG_TIDY "Turn on clang_tidy processing if available" ON)
find_program(CLANG_TIDY_EXE NAMES "clang-tidy")
if(CLANG_TIDY_EXE)
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    if(NOT CLANG_TIDY)
        message(STATUS "clang-tidy NOT ENABLED via 'CLANG_TIDY' variable!")
        set(CMAKE_CXX_CLANG_TIDY "" CACHE STRING "" FORCE) # delete it
    endif()
elseif(CLANG_TIDY)
    message(SEND_ERROR "Cannot enable clang-tidy, as executable not found!")
    set(CMAKE_CXX_CLANG_TIDY "" CACHE STRING "" FORCE) # delete it
else()
    message(STATUS "clang-tidy not found!")
    set(CMAKE_CXX_CLANG_TIDY "" CACHE STRING "" FORCE) # delete it
endif()
if(CLANG_TIDY AND CLANG_TIDY_EXE)
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}" "-fix")
endif()
