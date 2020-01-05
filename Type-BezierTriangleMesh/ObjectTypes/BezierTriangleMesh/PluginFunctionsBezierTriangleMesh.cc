//=============================================================================
//
//  Plugin Functions
//
//=============================================================================

#include <OpenFlipper/common/Types.hh>

#include "BezierTriangleMesh.hh"
#include <OpenFlipper/BasePlugin/PluginFunctions.hh>

namespace PluginFunctions {

void betriOption(betri::BezierOption option, int value)
{
	betri::option(option, value);
}

int betriOption(betri::BezierOption option)
{
	return betri::option(option);
}

bool getSourceMeshes(std::vector<BezierTMesh*> &_meshes)
{
	_meshes.clear();

	for (ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
		o_it != PluginFunctions::objectsEnd(); ++o_it
	) {
		if (!o_it->source())
			continue;
		_meshes.push_back(dynamic_cast<BTMeshObject*>(*o_it)->mesh());
	}

	return (!_meshes.empty());
}

bool getTargetMeshes(std::vector<BezierTMesh*> &_meshes)
{
	_meshes.clear();

	for (ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
		o_it != PluginFunctions::objectsEnd(); ++o_it
	) {
		if (!o_it->target())
			continue;
		if (dynamic_cast<BTMeshObject*>(*o_it)->mesh())
			_meshes.push_back(dynamic_cast<BTMeshObject*>(*o_it)->mesh());
	}

	return (!_meshes.empty());
}


bool getObject(int _identifier, BTMeshObject *&_object)
{
	if (_identifier == BaseObject::NOOBJECT) {
		_object = 0;
		return false;
	}

	// get object by using the map accelerated plugin function
	BaseObjectData* object = 0;
	PluginFunctions::getObject(_identifier, object);

	_object = dynamic_cast<BTMeshObject*>(object);
	return (_object != 0);
}

// ===============================================================================
// ===============================================================================


bool getMesh(int _identifier, BezierTMesh *&_mesh)
{
	if (_identifier == BaseObject::NOOBJECT) {
		_mesh = 0;
		return false;
	}

	// get object by using the map accelerated plugin function
	BaseObjectData* object = 0;
	PluginFunctions::getObject(_identifier, object);

	// Unable to find object
	if (object == 0)
		return false;

	BTMeshObject* triangleMeshObject = dynamic_cast<BTMeshObject*> (object);

	// Object is not a triangle mesh
	if (triangleMeshObject == 0)
		return false;

	_mesh = triangleMeshObject->mesh();
	return true;
}


// ===============================================================================
// Getting data from objects and casting between them
// ===============================================================================

BezierTMesh* btMesh(BaseObjectData* _object)
{
	if (_object == 0)
		return 0;

	if (_object->dataType(DATA_BEZIER_TRIANGLE_MESH)) {
		BTMeshObject* object = dynamic_cast<BTMeshObject*>(_object);
		return object->mesh();
	}

	return 0;
}

BezierTMesh* btMesh(int _identifier)
{
	BTMeshObject* object = btMeshObject(_identifier);

	return object == 0 ? 0 : object->mesh();
}

BTMeshObject* btMeshObject(BaseObjectData* _object)
{
	if (_object == 0)
		return 0;

	if (!_object->dataType(DATA_BEZIER_TRIANGLE_MESH))
		return 0;

	return dynamic_cast<BTMeshObject*>(_object);
}


BTMeshObject* btMeshObject(int _objectId)
{
	if (_objectId == BaseObject::NOOBJECT)
		return 0;

	// Get object by using the map accelerated plugin function
	BaseObjectData* object = 0;
	PluginFunctions::getObject(_objectId, object);

	if (object == 0)
		return 0;

	BTMeshObject* meshObject = dynamic_cast<BTMeshObject*>(object);

	return meshObject;
}


}