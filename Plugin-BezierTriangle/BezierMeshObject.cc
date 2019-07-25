#include "BezierMeshObject.hh"


BezierMeshObject::BezierMeshObject(const BezierMeshObject& _object) : MeshObject<BezierTriangleMesh>(_object) {

}


BezierMeshObject::BezierMeshObject(DataType _typeId) : MeshObject<BezierTriangleMesh>(_typeId) {

}


BezierMeshObject::~BezierMeshObject() {

}

BaseObject* BezierMeshObject::copy() {
	BezierMeshObject* object = new BezierMeshObject(*this);
	return object;
}