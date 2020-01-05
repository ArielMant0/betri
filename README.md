# betri
OpenFlipper-Plugins to represent, create and display Bézier triangles

## Plugins & Usage

### BezierTriangleMesh (necessary)
This plugin contains the data structure for the Bezier triangle mesh.

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
- Convex Hull (does not work for every mesh)
- Bounding Billboard

### BezierTriangleAlgorithms
This plugin contains the Voronoi and decimation meshing algorithms.
The first number box defines the Bézier degree for the meshing algorithm.

For the Voronoi meshing, the complete algorithm can be performed using the
*Voronoi meshing* button, or the single steps can be performed using the buttons below.
Note: No checks are done to prevent you from clicking the buttons in the wrong order,
just go from left to right or live with the application crashing at some point.
When starting the Voronoi algorithm, you will be asked for a minimum partition
number. This does not always work reliably for an arbitrary number, so specify a
minimum of 10 if you want to make sure it works.

- Voronoi meshing parameters
    - **use colors**: display partition etc. using face and vertex colors
    - **interpolate**: don't fix control points but interpolate them after fitting
    - **overwrite mesh**: update the original mesh, else a control mesh contains the result
    - **parametrization weights**: the weights that should be used for the parametrization step, with cotangent usually producing better results

The decimation algorithm can also be performed in a stepwise fashion. When the algorithm
is started, you will be asked for the desired target vertex complexity the mesh should be
reduced tom starting with a suggestion of the number of vertices present in the mesh.
- Decimation meshing parameters
    - **display error**: displays the fitting error using vertex colors
    - **interpolate**: don't fix control points but interpolate them after fitting

### FileBTM
This plugin can read and write .btm files, which simply contain vertex and face data
together with the control points.

### FileBTOFF
This plugin reads and writes .off files to produce Bezier triangle meshes.

### BezierTriangleUtils
This plugin contains some utility functionalities that may be useful:
- **Tessellate mesh**: tessellates the target mesh by the desired amount
- **Tessellate to trimesh**: tessellates the source mesh to a new triangle mesh by the desired amount
- **Randomize Control Points**: useful for testing, shifts all control points except
for the corners along the interpolated vertex normals, not really random or anything
- **Add texture coordinates**: adds some texture coordinates to the mesh

## Setup

This section explains how the get these plugins working.

### Notes
These may not be necessary for any pc configuration, but were required for me.

- Make sure a new version of Qt is installed which includes **QT_NO_FLOAT16_OPERATORS**
    - can be added to cmake file with `add_compile_definitions(QT_NO_FLOAT16_OPERATORS)`
    macro so there are no problems with the std::bitset header
- Make sure this macro is defined in all relevant projects (OpenFlipper, OpenFlipperLib and all plugins)

### Building Plugins
1. Download source code found at **release**
2. Make sure dependencies are present and installed
3. Copy plugin folders (as you require) such that the following file structure is given
    - `OpenFlipper/Plugin-BezierTriangleAlgorithms`
    - `OpenFlipper/Plugin-BezierTriangleRendering`
    - `OpenFlipper/Plugin-BezierTriangleUtils`
    - `OpenFlipper/Type-BezierTriangleMesh`
    - `OpenFlipper/PluginCollections-FilePlugins/Plugin-FileBTM`
    - `OpenFlipper/PluginCollections-FilePlugins/Plugin-FileBTOFF`
4. Follow OpenFlipper build instructions

## Benchmarking

### Algorithms

TODO

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
