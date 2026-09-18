project(goldsrc)

# Collect all source files
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

# The conversion pipeline's identity, hashed from the sources above and regenerated whenever any
# of them changes. GoldSrcBSP hands it to GDScript, which bakes it into every .scn it converts and
# refuses a cached map that was built by a different one — the C++ half of this library decides
# what a .scn contains, and a cache version living in GDScript cannot see that half move.
#
# Every source counts, not just the BSP path. Over-invalidating costs a reconversion; under-
# invalidating ships a map built by code nobody is running any more, silently.
set(PIPELINE_ID_HEADER "${CMAKE_CURRENT_BINARY_DIR}/goldsrc_pipeline_id.h")
add_custom_command(
    OUTPUT "${PIPELINE_ID_HEADER}"
    COMMAND ${CMAKE_COMMAND}
        -DGOLDSRC_ID_OUT=${PIPELINE_ID_HEADER}
        "-DGOLDSRC_ID_SOURCES=${SOURCES}"
        -P "${CMAKE_SOURCE_DIR}/cmake/pipeline_id.cmake"
    DEPENDS ${SOURCES} "${CMAKE_SOURCE_DIR}/cmake/pipeline_id.cmake"
    COMMENT "Hashing the conversion pipeline"
    VERBATIM
)

# Shared library for the GDExtension
add_library(${PROJECT_NAME} SHARED ${SOURCES} "${PIPELINE_ID_HEADER}")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE godot::cpp)

# Output naming: libgoldsrc.<platform>.template_<config>.<arch>
string(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME)
if(SYSTEM_NAME STREQUAL "darwin")
    set(SYSTEM_NAME "macos")
endif()
# Allow toolchain files to override platform name (e.g. "android" when CMAKE_SYSTEM_NAME=Linux)
if(GOLDSRC_PLATFORM_NAME)
    set(SYSTEM_NAME "${GOLDSRC_PLATFORM_NAME}")
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

# Output to the addon's bin directory
set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/addons/goldsrc/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/addons/goldsrc/bin"
)
