#pragma once

//== INCLUDES =================================================================

#include <ObjectTypes/TriangleMesh/TriangleMeshTypes.hh>

struct BezierTriangleTraits : public TriTraits
{
	EdgeTraits{
		size_t curveIndex;
	};

	FaceTraits{
		size_t bezierIndex;
	};
};

//== TYPEDEFS =================================================================

using BezierTMesh = OpenMesh::TriMesh_ArrayKernelT<BezierTriangleTraits>;
