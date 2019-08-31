#include "BezierTriangleUtils.hh"
#include "algorithms/VoronoiRemeshPerObjectData.hh"

using VOD = VoronoiRemeshPerObjectData;

namespace betri
{

const char * VODName()
{
	return "VORONOIREMESH_PER_OBJECT_DATA";
}

VoronoiRemesh* getVoronoiObject(BaseObjectData *object)
{
	// initialize PerObjectData if not done yet
	if (!object->hasObjectData(VODName()))
	{
		// get mesh object
		BezierTMesh* mesh = dynamic_cast<BTMeshObject*>(object)->mesh();

		// initialize per object data
		object->setObjectData(VODName(), new VOD(*mesh));
	}

	// get feature lines object
	VoronoiRemesh* remesher = dynamic_cast<VoronoiRemesh*>(
		&(dynamic_cast<VOD*>(object->objectData(VODName())))->remesher()
	);

	return remesher;
}

void voronoiRemesh(BaseObjectData *object)
{
	BezierTMesh *mesh = dynamic_cast<BTMeshObject*>(object)->mesh();
	getVoronoiObject(object)->remesh(mesh->n_faces() * 0.2);
}

void decimate(BaseObjectData *object) {}

}
