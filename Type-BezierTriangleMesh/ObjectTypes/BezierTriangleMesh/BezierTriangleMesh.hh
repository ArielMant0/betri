#pragma once
/**
 * \file BezierTriangleMesh.hh
 * This File contains all required includes for using BezierTriangleMeshes
*/

#define DATA_BEZIER_TRIANGLE_MESH typeId("BezierTriangleMesh")

//== INCLUDES =================================================================

#define DRAW_CURVED

#ifdef DRAW_CURVED
#include <ObjectTypes/BezierTriangleMesh/BTMeshObject.hh>
#else
#include <ObjectTypes/BezierTriangleMesh/BTMeshObject_tri.hh>
#endif

#include <ObjectTypes/BezierTriangleMesh/PluginFunctionsBezierTriangleMesh.hh>
#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMeshTypes.hh>

