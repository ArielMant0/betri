#ifndef BEZIER_MESH_OBJECT_INCLUDE_HH
#define BEZIER_MESH_OBJECT_INCLUDE_HH


//== INCLUDES =================================================================

#define DATA_BEZIER_TRIANGLE_MESH typeId("BezierTriangleMesh")
#include <ObjectTypes/MeshObject/MeshObjectT.hh>
#include "BezierTriangleMesh.hh"


class BezierMeshObject : public MeshObject<BezierTriangleMesh> {

public:
	BezierMeshObject(const BezierMeshObject& _object);

	BezierMeshObject(DataType _typeId);

	virtual ~BezierMeshObject();

	BaseObject* copy();

};

#endif // BEZIER_MESH_OBJECT_INCLUDE_HH defined