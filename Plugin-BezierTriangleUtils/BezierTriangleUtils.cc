#include "BezierTriangleUtils.hh"
#include "algorithms/VoronoiRemeshPerObjectData.hh"

using VOD = VoronoiRemeshPerObjectData;

namespace betri
{

const char * VODName()
{
	return "VORONOIREMESH_PER_OBJECT_DATA";
}

VoronoiRemesh* getVoronoiObject(BaseObjectData *object, BaseObjectData *ctrl)
{
	// initialize PerObjectData if not done yet
	if (!object->hasObjectData(VODName()))
	{
		// get mesh object
		BezierTMesh* mesh = dynamic_cast<BTMeshObject*>(object)->mesh();
		BezierTMesh* ctrlMesh = dynamic_cast<BTMeshObject*>(ctrl)->mesh();


		// initialize per object data
		object->setObjectData(VODName(), new VOD(*mesh, *ctrlMesh));
	}

	// get feature lines object
	VoronoiRemesh* remesher = dynamic_cast<VoronoiRemesh*>(
		&(dynamic_cast<VOD*>(object->objectData(VODName())))->remesher()
	);

	return remesher;
}

void voronoiRemesh(BaseObjectData *object, BaseObjectData *ctrl, bool useColors)
{
	BezierTMesh *mesh = dynamic_cast<BTMeshObject*>(object)->mesh();
	getVoronoiObject(object, ctrl)->remesh(mesh->n_faces() * 0.2);

	ctrl->setObjectDrawMode(ACG::SceneGraph::DrawModes::HIDDENLINE);
	if (useColors) {
		object->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_FACES_COLORED |
			ACG::SceneGraph::DrawModes::EDGES_COLORED
		);
	}
}

void decimate(BaseObjectData *object) {}

}