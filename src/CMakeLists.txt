project(goldsrc)

# Collect all source files
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

# Shared library for the GDExtension
add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE godot::cpp)

# Output naming: libgoldsrc.<platform>.template_<config>.<arch>
string(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME)
if(SYSTEM_NAME STREQUAL "darwin")
    set(SYSTEM_NAME "macos")
endif()
string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE)

if(BUILD_TYPE STREQUAL "")
    set(BUILD_TYPE "debug")
endif()

# Map cmake build type to godot convention
if(BUILD_TYPE STREQUAL "debug" OR BUILD_TYPE STREQUAL "relwithdebinfo")
    set(GODOT_BUILD_TYPE "template_debug")
else()
    set(GODOT_BUILD_TYPE "template_release")
endif()

# Determine architecture string
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ARCH "arm64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
    set(ARCH "x86_64")
else()
    set(ARCH ${CMAKE_SYSTEM_PROCESSOR})
endif()

set(OUTPUT_NAME "goldsrc.${SYSTEM_NAME}.${GODOT_BUILD_TYPE}.${ARCH}")

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
    OUTPUT_NAME "${OUTPUT_NAME}"
    PREFIX "lib"
)

# Output directly to demo addon so Godot can find it
set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/demo/addons/goldsrc/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/demo/addons/goldsrc/bin"
)
