#include "BezierTriangleAlgorithms.hh"
#include "voronoi/VoronoiRemeshPerObjectData.hh"
#include "decimation/DecimationPerObjectData.hh"

#include "voronoi/VoronoiFitting.hh"
#include "voronoi/VoronoiParametrization.hh"

#include "decimation/DecimationParametrization.hh"
#include "decimation/DecimationFitting.hh"

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
void voronoiInit(
	BaseObjectData *object,
	BaseObjectData *ctrl,
	size_t count,
	const bool useColors,
	const bool interpolate
) {
	auto remesher = getVoronoiObject(object, ctrl);
	remesher->minPartition(count);
	remesher->useColors(useColors);
	remesher->interpolate(interpolate);
}

void voronoiRemesh(BaseObjectData *object, BaseObjectData *ctrl)
{
	auto remesher = getVoronoiObject(object, ctrl);

	remesher->remesh();

	if (remesher->useColors()) {
		object->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED
		);
		ctrl->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED
		);
	}
}

void voronoiPartition(BaseObjectData *object, BaseObjectData *ctrl)
{
	auto remesher = getVoronoiObject(object, ctrl);
	remesher->partition();

	if (remesher->useColors()) {
		object->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_FACES_COLORED |
			ACG::SceneGraph::DrawModes::WIREFRAME
		);
	}
}

bool voronoiDual(BaseObjectData *object, BaseObjectData *ctrl, bool steps)
{

	auto remesher = getVoronoiObject(object, ctrl);
	bool done = remesher->dualize(steps);

	if (remesher->useColors()) {
		object->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_POINTS_COLORED |
			ACG::SceneGraph::DrawModes::WIREFRAME
		);
	}

	return done;
}

void voronoiFitting(BaseObjectData *object, BaseObjectData *ctrl)
{
	auto remesher = getVoronoiObject(object, ctrl);
	remesher->fitting();

	BezierTMesh *mesh = PluginFunctions::btMeshObject(object)->mesh();
	mesh->garbage_collection();

	object->setObjectDrawMode(
		ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED
	);
	ctrl->setObjectDrawMode(
		ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED
	);
}

void voronoiFittingTest(BaseObjectData *object, BaseObjectData *ctrl)
{
	BezierTMesh *mesh = PluginFunctions::btMeshObject(object)->mesh();
	BezierTMesh *ctrlMesh = PluginFunctions::btMeshObject(ctrl)->mesh();

	object->setObjectData(VODName(), new VOD(*mesh, *ctrlMesh));
	VoronoiRemesh* remesher = dynamic_cast<VoronoiRemesh*>(
		&(dynamic_cast<VOD*>(object->objectData(VODName())))->remesher()
	);
	remesher->useBaseMesh(true);
	remesher->remesh();

	mesh->garbage_collection();
	ctrlMesh->garbage_collection();

	object->setObjectDrawMode(
		ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED |
		ACG::SceneGraph::DrawModes::WIREFRAME
	);
	ctrl->setObjectDrawMode(
		ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED |
		ACG::SceneGraph::DrawModes::WIREFRAME
	);
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

void decimationInit(BaseObjectData *object, size_t complexity, bool color)
{
	auto decimator = getDecimationObject(object);
	decimator->initialize(complexity);
	decimator->useColors(color);

	if (color) {
		object->setObjectDrawMode(ACG::SceneGraph::DrawModes::SOLID_POINTS_COLORED);
	}
}

bool decimation(BaseObjectData *object, bool steps, bool interpolate)
{
	auto decimator = getDecimationObject(object);
	decimator->interpolate(interpolate);

	bool done = decimator->decimate(steps);

	if (decimator->useColors()) {
		object->setObjectDrawMode(ACG::SceneGraph::DrawModes::SOLID_POINTS_COLORED);
	}

	return done;
}

bool test(TestOptions which, BezierTMesh *mesh)
{
	switch (which) {
		case TestOptions::voronoi_fit: return VoronoiFitting::test(mesh);
		case TestOptions::voronoi_param: return VoronoiParametrization::test(mesh);
		case TestOptions::decimation_param: return DecimationParametrization::test(mesh);
		case TestOptions::decimation_fit: return DecimationFitting::test(mesh);
		default: return true;
	}
}

}