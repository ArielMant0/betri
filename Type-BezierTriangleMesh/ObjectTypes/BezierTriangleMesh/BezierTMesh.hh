#include "BezierTriangleMeshTypes.hh"

#include <OpenFlipper/common/ObjectTypeDLLDefines.hh>

class OBJECTTYPEDLLEXPORTONLY BezierTMesh : public OpenMesh::TriMesh_ArrayKernelT<BezierTriangleTraits>
{
public:

	void degreeElevation(BezierTMesh::FaceHandle &fh);

	void degreeReduction(BezierTMesh::FaceHandle &fh);

	void addCPsToFace(FaceHandle &f, unsigned int degree=2);

	void addCPsToFace(const FaceHandle &f, unsigned int degree=2);

};