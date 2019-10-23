include (plugin)
openflipper_plugin (
    DIRS ./algorithms/common ./algorithms/voronoi ./algorithms/decimation
    DEPS EIGEN3
    INSTALLDATA Icons
    TYPES BEZIERTRIANGLEMESH
)
