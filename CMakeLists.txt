cmake_minimum_required(VERSION 3.22)

project(goldsrc-godot
    LANGUAGES CXX
    VERSION 0.1.0
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# godot-cpp (git submodule at extern/godot-cpp)
add_subdirectory(extern/godot-cpp)

# Our GDExtension library
add_subdirectory(src)

# Unit tests (pure C++, no Godot dependency)
enable_testing()
add_subdirectory(tests)
