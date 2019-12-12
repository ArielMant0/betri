# betri
OpenFlipper-Plugin to create and display Bézier triangles

## Setup

- Make sure new Version of Qt is installed which includes **QT_NO_FLOAT16_OPERATORS**
    - can be added to cmake file with `add_compile_definitions(QT_NO_FLOAT16_OPERATORS)`
macro so there are no problems with the bitset header.
- Make sure this macro is defined in all relevant VS projects (OpenFlipper, OpenFlipperLib and all plugins)
- Make sure to use the correct cmake config for VS x64

## Structure

**Plugin-BezierTriangleUtils**
- this should be used for all functionalities that require user interaction, so all algorithms
for meshing or plugin configuration
- currently holds 2 functions, one that adds the bezier control points to a BezierTriangleMesh and
one that performs a basic algorithm to be used for meshing in the future

**Type-BezierTriangleMesh**
- this plugin defines the bezier triangle mesh type
- rendering code should *probably* also go here (copy paste from **BSplineCurve**)
- the "ObjectTypes" files can be located in the project **BEZIERTRIANGLEMESH**

**Plugin-FileBTOFF**
- this is a copy of the .off loader plugin that loads an .off file but as a bezier triangle mesh
- we should probably call the "addBezierTriangles" method here it does not need to be called manually
via the Utils plugin

## Compiling

If you add new files to any of the plugins or add a new plugin, you first need to rerun cmake
- go to the build folder (miniknight: "OpenFlipper/build", potatopc: "OpenFlipper/build/release")
- run "cmake-gui ..\.." in cmd (shortcur for cmd: press on path bar and type 'cmd')
- press "configure"
- press "generate"
- open project or if it was still open there will be a dialog asking to reload where you simply
click on "yes all", "okay", sth like that

If you simply changed a file you can just build the project the file is in again (or even ALL BUILD)
if you're unceratin which one to build

## Benchmarking

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
