#include "BezierTriangleUtils.hh"
#include "algorithms/voronoi/VoronoiRemeshPerObjectData.hh"
#include "algorithms/decimation/DecimationPerObjectData.hh"

using VOD = VoronoiRemeshPerObjectData;
using DEC = DecimationPerObjectData;

namespace betri
{

const char * VODName()
{
	return "VORONOIREMESH_PER_OBJECT_DATA";
}

VoronoiRemesh* getVoronoiObject(BaseObjectData *object, BaseObjectData *ctrl)
{
	// initialize PerObjectData if not done yet
	if (!object->hasObjectData(VODName())) {
		// get mesh object
		BezierTMesh* mesh = PluginFunctions::btMeshObject(object)->mesh();
		BezierTMesh* ctrlMesh = PluginFunctions::btMeshObject(ctrl)->mesh();

		// initialize per object data
		object->setObjectData(VODName(), new VOD(*mesh, *ctrlMesh));
	}

	// get feature lines object
	VoronoiRemesh* remesher = dynamic_cast<VoronoiRemesh*>(
		&(dynamic_cast<VOD*>(object->objectData(VODName())))->remesher()
		);

	return remesher;
}

// should be called once to allow for stepwise execution
void voronoiInit(BaseObjectData *object, BaseObjectData *ctrl, bool useColors)
{
	auto remesher = getVoronoiObject(object, ctrl);
	remesher->useColors(useColors);
}

void voronoiRemesh(BaseObjectData *object, BaseObjectData *ctrl)
{
	auto remesher = getVoronoiObject(object, ctrl);

	remesher->remesh();

	BezierTMesh *mesh = PluginFunctions::btMeshObject(object)->mesh();
	mesh->garbage_collection();
}

void voronoiPartition(BaseObjectData *object, BaseObjectData *ctrl)
{
	auto remesher = getVoronoiObject(object, ctrl);
	remesher->partition();

#ifndef DRAW_CURVED
	if (remesher->useColors()) {
		object->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_FACES_COLORED |
			ACG::SceneGraph::DrawModes::WIREFRAME
		);
	}
#endif

}

bool voronoiDual(BaseObjectData *object, BaseObjectData *ctrl, bool steps)
{

	auto remesher = getVoronoiObject(object, ctrl);
	bool done = remesher->dualize(steps);

#ifndef DRAW_CURVED
	if (remesher->useColors()) {
		object->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_POINTS_COLORED |
			ACG::SceneGraph::DrawModes::WIREFRAME
			//| ACG::SceneGraph::DrawModes::EDGES_COLORED
		);
	}
#endif

	return done;
}

void voronoiFitting(BaseObjectData *object, BaseObjectData *ctrl)
{
	auto remesher = getVoronoiObject(object, ctrl);
	remesher->fitting();

	BezierTMesh *mesh = PluginFunctions::btMeshObject(object)->mesh();
	mesh->garbage_collection();

#ifndef DRAW_CURVED
	if (remesher->useColors()) {
		object->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_SMOOTH_SHADED
		);
		ctrl->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_SMOOTH_SHADED
		);
	}
#endif
}

const char * DECName()
{
	return "DECIMATION_PER_OBJECT_DATA";
}

Decimation* getDecimationObject(BaseObjectData *object)
{
	// initialize PerObjectData if not done yet
	if (!object->hasObjectData(DECName())) {
		// get mesh object
		BezierTMesh* mesh = PluginFunctions::btMeshObject(object)->mesh();

		// initialize per object data
		object->setObjectData(DECName(), new DEC(*mesh));
	}

	// get feature lines object
	Decimation* decimator = dynamic_cast<Decimation*>(
		&(dynamic_cast<DEC*>(object->objectData(DECName())))->decimator()
	);

	return decimator;
}

bool decimation(BaseObjectData *object, size_t complexity, bool steps)
{
	return getDecimationObject(object)->decimate(complexity, steps);
}

}
