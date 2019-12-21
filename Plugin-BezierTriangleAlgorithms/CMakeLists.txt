include (plugin)
openflipper_plugin (
    DIRS ./common ./voronoi ./decimation
    DEPS EIGEN3
    INSTALLDATA Icons
    TYPES BEZIERTRIANGLEMESH
)

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    add_compile_definitions(BEZIER_DEBUG)
endif()
