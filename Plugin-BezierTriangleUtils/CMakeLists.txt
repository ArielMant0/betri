include (plugin)
openflipper_plugin (
    DIRS ./algorithms/common ./algorithms/voronoi ./algorithms/decimation
    DEPS EIGEN3
    INSTALLDATA Icons
    TYPES BEZIERTRIANGLEMESH TRIANGLEMESH
)

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    add_compile_definitions(BEZIER_DEBUG)
endif()
