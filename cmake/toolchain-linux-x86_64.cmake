set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# Wrapper scripts call zig cc/c++ with the right target triple.
# glibc 2.28 matches Godot 4.x's official Linux build baseline (Ubuntu 18.04).
set(CMAKE_C_COMPILER   "${CMAKE_CURRENT_LIST_DIR}/zig-cc-linux.sh")
set(CMAKE_CXX_COMPILER "${CMAKE_CURRENT_LIST_DIR}/zig-cxx-linux.sh")
set(CMAKE_AR           "${CMAKE_CURRENT_LIST_DIR}/zig-ar.sh")
set(CMAKE_RANLIB       "${CMAKE_CURRENT_LIST_DIR}/zig-ranlib.sh")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Prevent CMake from trying to run cross-compiled test binaries on the host.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
