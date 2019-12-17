#include "BezierTriangleUtils.hh"
#include "algorithms/voronoi/VoronoiRemeshPerObjectData.hh"
#include "algorithms/decimation/DecimationPerObjectData.hh"

#include "algorithms/voronoi/VoronoiFitting.hh"
#include "algorithms/voronoi/VoronoiParametrization.hh"

#include "algorithms/decimation/DecimationParametrization.hh"
#include "algorithms/decimation/DecimationFitting.hh"

using VOD = VoronoiRemeshPerObjectData;
using DEC = DecimationPerObjectData;

namespace betri
{

void randomMeshUV(BezierTMesh &mesh)
{
	mesh.request_vertex_texcoords2D();
	for (auto f_it = mesh.faces_sbegin(); f_it != mesh.faces_end(); ++f_it) {
		float tcU = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (1.0 - 0.0)));
		float tcV = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (1.0 - 0.0)));
		float angle = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * M_PI - 0.0)));

		auto fv_it = mesh.fv_iter(*f_it);
		auto vh0 = *fv_it++;
		auto vh1 = *fv_it++;
		auto vh2 = *fv_it;

		// TODO shorten this method

		// Fill all three if they are empty
		if (mesh.texcoord2D(vh0)[0] < -1000.0 && mesh.texcoord2D(vh1)[0] < -1000.0 && mesh.texcoord2D(vh2)[0] < -1000.0) {
			//BezierTMesh::TexCoord2D tc(tcU, tcV);
			BezierTMesh::TexCoord2D tcZero(0.0, 0.0);
			BezierTMesh::TexCoord2D tcZDir(1.0, 0.0);
			mesh.set_texcoord2D(vh0, tcZero);

			auto vecV01 = mesh.point(vh1) - mesh.point(vh0);
			auto vecV01_n = vecV01;
			auto vecV02 = mesh.point(vh2) - mesh.point(vh0);
			auto vecV02_n = vecV02;
			auto angle = acos(dot(normalize(vecV01_n), normalize(vecV02_n)));

			mesh.set_texcoord2D(vh1, tcZDir * length(vecV01));
			auto tc = BezierTMesh::TexCoord2D(
				cos(angle) * tcZDir[0] - sin(angle) * tcZDir[1],
				sin(angle) * tcZDir[0] + cos(angle) * tcZDir[1]
			) * length(vecV02);
			mesh.set_texcoord2D(vh2, tc);
		}
		// Fill two if they are empty
		else if (mesh.texcoord2D(vh0)[0] > -1000.0 ^ mesh.texcoord2D(vh1)[0] > -1000.0 ^ mesh.texcoord2D(vh2)[0] > -1000.0) {
			continue;
			BezierTMesh::VertexHandle first;
			BezierTMesh::VertexHandle second;
			BezierTMesh::VertexHandle third;

			if (mesh.texcoord2D(vh0)[0] > -1000.0) {
				first = vh0;
				second = vh1;
				third = vh2;
			} else if (mesh.texcoord2D(vh1)[0] > -1000.0) {
				first = vh1;
				second = vh2;
				third = vh0;
			} else if (mesh.texcoord2D(vh2)[0] > -1000.0) {
				first = vh2;
				second = vh0;
				third = vh1;
			}

			BezierTMesh::TexCoord2D tcSecond(1.0, 0.0);
			mesh.set_texcoord2D(second, tcSecond + mesh.texcoord2D(first));

			auto vecV01 = mesh.point(second) - mesh.point(first);
			auto vecV01_n = vecV01;
			auto vecV02 = mesh.point(third) - mesh.point(first);
			auto vecV02_n = vecV02;
			auto angle = acos(dot(normalize(vecV01_n), normalize(vecV02_n)));

			auto tc = BezierTMesh::TexCoord2D(
				cos(angle) * tcSecond[0] - sin(angle) * tcSecond[1],
				sin(angle) * tcSecond[0] + cos(angle) * tcSecond[1]
			) * length(vecV02) + mesh.texcoord2D(first);
			mesh.set_texcoord2D(third, tc);
		}
		// Fill one if it is empty
		else if (mesh.texcoord2D(vh0)[0] < -1000.0 ^ mesh.texcoord2D(vh1)[0] < -1000.0 ^ mesh.texcoord2D(vh2)[0] < -1000.0) {

			BezierTMesh::VertexHandle first;
			BezierTMesh::VertexHandle second;
			BezierTMesh::VertexHandle third;

			if (mesh.texcoord2D(vh0)[0] < -1000.0) {
				first = vh1;
				second = vh2;
				third = vh0;
			} else if (mesh.texcoord2D(vh1)[0] < -1000.0) {
				first = vh2;
				second = vh0;
				third = vh1;
			} else if (mesh.texcoord2D(vh2)[0] < -1000.0) {
				first = vh0;
				second = vh1;
				third = vh2;
			}

			BezierTMesh::TexCoord2D tcSecond = mesh.texcoord2D(second) - mesh.texcoord2D(first);

			auto vecV01 = mesh.point(second) - mesh.point(first);
			auto vecV01_n = vecV01;
			auto vecV02 = mesh.point(third) - mesh.point(first);
			auto vecV02_n = vecV02;
			auto angle = acos(dot(normalize(vecV01_n), normalize(vecV02_n)));

			auto tc = BezierTMesh::TexCoord2D(
				cos(angle) * tcSecond[0] - sin(angle) * tcSecond[1],
				sin(angle) * tcSecond[0] + cos(angle) * tcSecond[1]
			) * length(vecV02) + mesh.texcoord2D(first);
			mesh.set_texcoord2D(third, tc);
		}
	}
}

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
void voronoiInit(BaseObjectData *object, BaseObjectData *ctrl, size_t count, bool untwist, bool useColors)
{
	auto remesher = getVoronoiObject(object, ctrl);
	remesher->minPartition(count);
	remesher->useColors(useColors);
	remesher->untwist(untwist);
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

void voronoiFitting(BaseObjectData *object, BaseObjectData *ctrl, bool untwist)
{
	auto remesher = getVoronoiObject(object, ctrl);
	remesher->untwist(untwist);
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

void voronoiSmooth(BaseObjectData *object, BaseObjectData *ctrl)
{
	auto remesher = getVoronoiObject(object, ctrl);
	remesher->smooth();
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

bool decimation(BaseObjectData *object, bool steps, bool untwist)
{
	auto decimator = getDecimationObject(object);
	decimator->untwist(untwist);

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
