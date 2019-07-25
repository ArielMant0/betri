#pragma once

//== INCLUDES =================================================================

//#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <Type-OpenMesh/ObjectTypes/TriangleMesh/TriangleMeshTypes.hh>
//#include <OpenFlipper/libs_required/ACG/Math/BezierCurveT.hh>

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
