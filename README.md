# betri
OpenFlipper-Plugins to represent, create and display Bézier triangles

## Plugins & Usage

### BezierTriangleMesh (necessary)
This plugin contains the data structure for the Bezier triangle mesh called *BezierTMesh*.

### BezierTriangleRendering (necessary)
This plugin handles all rendering and lets you configure how to render
Bézier triangle meshes.

**Render Modes**
- CPU Tessellation: tessellate mesh on the CPU
- GPU Tessellation: tessellate mesh on the GPU
- Ray Casting: uses a ray casting approach with the desired bounding volume

**Bounding Volumes**
- AABB (axis-aligned bounding boxes)
- Prism
- Tetrahedron
- Convex Hull
- Bounding Billboard

### BezierTriangleAlgorithms
This plugin contains the Voronoi and decimation meshing algorithms.

The first box defines the target Bézier triangle degree for both meshing algorithm.
The checkbox **time algorithm** gives the option of writing execution times
into a file *voronoi-times.txt* or *decimation-times.txt* that is saved at the location
where the last mesh was opened.

For the Voronoi meshing, the complete algorithm can be performed using the
*Voronoi meshing* button, or the single steps can be performed using the buttons below.
Note: No checks are done to prevent you from clicking the buttons in the wrong order,
just go from left to right or be responsible for the application crashing.

When starting the Voronoi algorithm, you will be asked for a minimum partition
number. This does not always work reliably for an arbitrary number, so specify a
very small number (or 0) if you want to make sure it works.

- Voronoi meshing parameters
    - **use colors**: display partition etc. using face and vertex colors
    - **interpolate**: don't fix control points but interpolate them after fitting
    - **overwrite mesh**: update the original mesh, else a control mesh contains the result
    - **parametrization weights**: the weights that should be used for the parametrization step, with cotangent usually producing better results
    - **max fitting samples**: maximum number of samples to use for fitting, set to 0 if all samples should be used
    - **fitting solver**: which solver should be used (adaptive uses QR decomposition for more than 100 samples)

The decimation algorithm can also be performed in a stepwise fashion. When the algorithm
is started, you will be asked for the desired target vertex complexity the mesh should be
reduced to, starting with a suggestion of the number of vertices the mesh currently has.
- Decimation meshing parameters
    - **display error**: displays the fitting error using vertex colors
    - **interpolate**: don't fix control points but interpolate them after fitting
    - **fitting samples**: number of samples to use for fitting
    - **fitting solver**: which solver should be used (adaptive uses QR decomposition for more than 100 samples)

### FileBTM
This plugin can read and write .btm files, which simply contains vertex and face data
together with the control points.

### FileBTOFF
This plugin reads .off files to produce Bezier triangle meshes.

**WARNING**: this plugin can not yet write/export files (this causes a crash)

### BezierTriangleUtils
This plugin contains some utility functionalities that may be useful:
- **Tessellate mesh**: tessellates the target mesh by the desired amount
- **Tessellate to trimesh**: tessellates the source mesh to a new triangle mesh by the desired amount
- **Shift Control Points**: useful for testing, shifts all control points except
for the corners along the interpolated vertex normals
- **Add texture coordinates**: adds some texture coordinates to the mesh

## Setup

This section explains how the get these plugins working.
This project has only been tested on Windows 10 build should also work for
Linux and Mac OS.

Should you encounter any problems, please contact [Franziska Becker](mailto:fbecker@uos.de)
or [René Warnking](mailto:rwarnking@uos.de).

### Dependencies

- [CMake](https://cmake.org)
- [OpenFlipper](http://openflipper.org)
- [OpenMesh](https://www.openmesh.org) (contained in OpenFlipper)
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- OpenGL version 4 or higher (4.3 or higher for OpenGL queries)

### Notes
These may not be necessary for any pc configuration, but were required for me.

- Make sure a new version of Qt is installed which includes **QT_NO_FLOAT16_OPERATORS**
    - can be added to cmake file with `add_compile_definitions(QT_NO_FLOAT16_OPERATORS)`
    macro so there are no problems with the std::bitset header
- Make sure this macro is defined in all relevant projects (OpenFlipper, OpenFlipperLib and all plugins)

### Building Plugins
1. Download source code found at the **release** page
2. Make sure dependencies are present and installed
3. Copy plugin folders (as you require) such that the following file structure is given
    - `OpenFlipper/Plugin-BezierTriangleAlgorithms`
    - `OpenFlipper/Plugin-BezierTriangleRendering`
    - `OpenFlipper/Plugin-BezierTriangleUtils`
    - `OpenFlipper/Type-BezierTriangleMesh`
    - `OpenFlipper/PluginCollections-FilePlugins/Plugin-FileBTM`
    - `OpenFlipper/PluginCollections-FilePlugins/Plugin-FileBTOFF`
4. Run CMake and set the path for Eigen
4. Follow further OpenFlipper build instructions

## Benchmarking

### Algorithms

Use the **time algorithm** option of the BezierAlgorithmsPlugin.

### Rendering

In order to benchmark the rendering via the interface provided in the plugin,
you need to edit the **IRenderer.cc** located in **libs_required/ACG/GL**.

Inside the method
```c++
void IRenderer::drawObject(ACG::RenderObject* _obj)
```

you need to surround the part you want to measure (e. g. `glDrawElementsInstanced`).

```c++
// start the benchmarking process
Benchmarker::instance()->startFrame()

// parts you want to measure

// end the benchmarking process
Benchmarker::instance()->endFrame()
```

## Documentation

Currently no public documentation is available, please look at the code
documentation or write an email if you need help.
