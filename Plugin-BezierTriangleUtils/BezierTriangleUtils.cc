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

void voronoiRemesh(BaseObjectData *object, BaseObjectData *ctrl, bool useColors, bool steps)
{
	BezierTMesh *mesh = dynamic_cast<BTMeshObject*>(object)->mesh();
	auto remesher = getVoronoiObject(object, ctrl);
	remesher->useColors(useColors);
	remesher->useSteps(steps);

	voronoiRemeshStep(object, ctrl, useColors);
}

void voronoiRemeshStep(BaseObjectData *object, BaseObjectData *ctrl, bool useColors)
{
	BezierTMesh *mesh = dynamic_cast<BTMeshObject*>(object)->mesh();
	auto remesher = getVoronoiObject(object, ctrl);
	remesher->remesh();

	ctrl->setObjectDrawMode(ACG::SceneGraph::DrawModes::HIDDENLINE);

	mesh->garbage_collection();

	if (useColors) {
		object->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_FACES_COLORED
			| ACG::SceneGraph::DrawModes::WIREFRAME
			// | ACG::SceneGraph::DrawModes::EDGES_COLORED
		);
	}
}

void decimate(BaseObjectData *object) {}

}
