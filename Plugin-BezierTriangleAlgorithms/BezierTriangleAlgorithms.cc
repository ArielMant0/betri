#include "BezierTriangleAlgorithms.hh"

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
	const bool interpolate,
	const bool overwrite,
	const bool splits,
	const int paramIndex,
	const size_t fittingSamples,
	const int fittingSolver
) {
	auto remesher = getVoronoiObject(object, ctrl);
	remesher->minPartition(count);
	remesher->useColors(useColors);
	remesher->interpolate(interpolate);
	remesher->overwrite(overwrite);
	remesher->weights(paramIndex);
	remesher->splits(splits);
	remesher->fittingSamples(fittingSamples);
	remesher->fittingSolver((Fitting::Solver)fittingSolver);
}

bool voronoiRemesh(BaseObjectData *object, BaseObjectData *ctrl)
{
	auto remesher = getVoronoiObject(object, ctrl);

	bool success = remesher->remesh();

	if (success) {
		object->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED
		);
		ctrl->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED
		);
	}

	return success;
}

bool voronoiPartition(BaseObjectData *object, BaseObjectData *ctrl, const bool steps, bool &done)
{
	auto remesher = getVoronoiObject(object, ctrl);
	bool success = remesher->partition(steps, done);

	if (success && remesher->useColors()) {
		object->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_FACES_COLORED |
			ACG::SceneGraph::DrawModes::WIREFRAME
		);
	}

	return success;
}

bool voronoiDual(BaseObjectData *object, BaseObjectData *ctrl, bool steps, bool &done)
{

	auto remesher = getVoronoiObject(object, ctrl);
	bool success = remesher->dualize(done, steps);

	if (success && remesher->useColors()) {
		object->setObjectDrawMode(
			ACG::SceneGraph::DrawModes::SOLID_POINTS_COLORED |
			ACG::SceneGraph::DrawModes::WIREFRAME
		);
	}

	return success;
}

bool voronoiFitting(BaseObjectData *object, BaseObjectData *ctrl)
{
	auto remesher = getVoronoiObject(object, ctrl);

	bool success = remesher->fitting();

	object->setObjectDrawMode(
		ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED
	);
	ctrl->setObjectDrawMode(
		ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED
	);

	return success;
}

VoronoiInfo voronoiInfo(BaseObjectData *object, BaseObjectData *ctrl)
{
	VoronoiInfo info;

	BezierTMesh* mesh = PluginFunctions::btMeshObject(object)->mesh();
	info.name = object->name().toStdString();
	info.vertices = std::to_string(mesh->n_vertices());
	info.edges = std::to_string(mesh->n_edges());
	info.faces = std::to_string(mesh->n_faces());

	auto remesher = getVoronoiObject(object, ctrl);
	info.partition = std::to_string(remesher->minPartition());

	return info;
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

void decimationInit(
	BaseObjectData *object,
	const size_t complexity,
	const size_t fittingSamples,
	const int fittingSolver,
	const bool color
) {
	auto decimator = getDecimationObject(object);
	decimator->initialize(complexity);
	decimator->useColors(color);
	decimator->fittingSamples(fittingSamples);
	decimator->fittingSolver((Fitting::Solver)fittingSolver);
}

bool decimation(BaseObjectData *object, const bool steps, const bool interpolate)
{
	auto decimator = getDecimationObject(object);
	decimator->interpolate(interpolate);

	bool done = decimator->decimate(steps);

	if (decimator->useColors()) {
		object->setObjectDrawMode(ACG::SceneGraph::DrawModes::SOLID_POINTS_COLORED);
	} else {
		object->setObjectDrawMode(ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED);
	}

	return done;
}

DecimationInfo decimationInfo(BaseObjectData * object)
{
	DecimationInfo info;

	BezierTMesh* mesh = PluginFunctions::btMeshObject(object)->mesh();
	info.name = object->name().toStdString();
	info.vertices = std::to_string(mesh->n_vertices());
	info.edges = std::to_string(mesh->n_edges());
	info.faces = std::to_string(mesh->n_faces());

	auto decimator = getDecimationObject(object);
	info.target = std::to_string(decimator->complexity());

	return info;
}

}
