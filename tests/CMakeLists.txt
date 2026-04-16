cmake_minimum_required(VERSION 3.22)
project(goldsrc-tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG        v2.4.11
)
FetchContent_MakeAvailable(doctest)

set(PARSER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../src/parsers")

add_executable(test_parsers
    test_parsers.cpp
    ${PARSER_DIR}/bsp_parser.cpp
    ${PARSER_DIR}/mdl_parser.cpp
    ${PARSER_DIR}/spr_parser.cpp
    ${PARSER_DIR}/wad_parser.cpp
)

target_include_directories(test_parsers PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

target_link_libraries(test_parsers PRIVATE doctest::doctest)
target_compile_features(test_parsers PRIVATE cxx_std_17)

enable_testing()
add_test(NAME test_parsers COMMAND test_parsers)
