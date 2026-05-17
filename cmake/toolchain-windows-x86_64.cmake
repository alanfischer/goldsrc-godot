set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# Wrapper scripts call zig cc/c++ with the right target triple.
# MinGW ABI: Godot loads GDExtensions via LoadLibraryW+GetProcAddress, no import lib needed.
set(CMAKE_C_COMPILER   "${CMAKE_CURRENT_LIST_DIR}/zig-cc-windows.sh")
set(CMAKE_CXX_COMPILER "${CMAKE_CURRENT_LIST_DIR}/zig-cxx-windows.sh")
set(CMAKE_AR           "${CMAKE_CURRENT_LIST_DIR}/zig-ar.sh")
set(CMAKE_RANLIB       "${CMAKE_CURRENT_LIST_DIR}/zig-ranlib.sh")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Prevent CMake from trying to run cross-compiled test binaries on the host.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
