#include "BezierTriangleUtilsPlugin.hh"

#include <qpushbutton.h>
#include <qgridlayout.h>
// TODO new
#include <qgroupbox.h>
#include <qpushbutton.h>
#include <qinputdialog.h>

#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <OpenFlipper/common/GlobalOptions.hh>
#include <OpenFlipper/common/RendererInfo.hh>

#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

#include <OpenFlipper/libs_required/OpenMesh/src/OpenMesh/Core/Utils/PropertyManager.hh>
#include "OpenFlipper/BasePlugin/PluginFunctions.hh"

void BezierTriangleUtilsPlugin::initializePlugin()
{
	m_tool = new QWidget();
	QIcon *toolIcon = new QIcon(
		OpenFlipper::Options::iconDirStr() +
		OpenFlipper::Options::dirSeparator() +
		"btutils.png"
	);

	QPushButton *applyTessTri = new QPushButton("Tessellate to TriMesh");
	QPushButton *applyTess = new QPushButton("Tessellate Mesh");
	QPushButton *randomCP = new QPushButton("Randomize Control Points");
	QPushButton *addTex = new QPushButton("Add Texture Coordinates");

	connect(applyTessTri, QOverload<>::of(&QPushButton::pressed),
		this, [&]() {
		applyTessellation(true);
	});
	connect(applyTess, QOverload<>::of(&QPushButton::pressed),
		this, [&]() {
		applyTessellation(false);
	});
	connect(randomCP, SIGNAL(clicked()), this, SLOT(addRandomizedControlPoints()));
	connect(addTex, SIGNAL(clicked()), this, SLOT(addTextureCoordinates()));

	///////////////////////////////////////////////////////////////////////////
	// Add all Elements
	///////////////////////////////////////////////////////////////////////////

	QGridLayout *grid = new QGridLayout();
	grid->addWidget(applyTess, 0, 0);
	grid->addWidget(applyTessTri, 1, 0);
	grid->addWidget(randomCP, 2, 0);
	grid->addWidget(addTex, 3, 0);
	m_tool->setLayout(grid);

    emit addToolbox(tr("Bezier Triangle Utils"), m_tool, toolIcon);
}

void BezierTriangleUtilsPlugin::applyTessellation(bool toTriMesh)
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::SOURCE_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);
		BezierTMesh *mesh = meshObj->mesh();

		bool okay = true;
		int amount = QInputDialog::getInt(
			m_tool,
			"Tessellation",
			"Please enter subdivision amount: ",
			// value, min value
			1, 1,
			// max value, steps
			5, 1, &okay
		);

		if (okay) {

			int tri_id = -1;

			if (toTriMesh) {

				TriMeshObject* object(0);

				emit addEmptyObject(DATA_TRIANGLE_MESH, tri_id);
				PluginFunctions::getObject(tri_id, object);

				mesh->tessellateToTrimesh(*object->mesh(), amount);

				emit updatedObject(tri_id, UPDATE_ALL);
			} else {
				mesh->tessellate(amount);

				emit updatedObject(meshObj->id(), UPDATE_ALL);
			}
		}
	}
}

void BezierTriangleUtilsPlugin::addRandomizedControlPoints()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);
		BezierTMesh *mesh = meshObj->mesh();

		for (BezierTMesh::FaceHandle fh : mesh->faces()) {
			mesh->recalculateCPs(fh, true);
		}

		emit updatedObject(meshObj->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleUtilsPlugin::addTextureCoordinates()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {

		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);
		BezierTMesh *mesh = meshObj->mesh();

		using Point = BezierTMesh::Point;
		using TexCoord2D = BezierTMesh::TexCoord2D;
		using VertexHandle = BezierTMesh::VertexHandle;

		if (!mesh->has_vertex_texcoords2D()) {
			mesh->request_vertex_texcoords2D();
		}

		Point min = Point(0.0);
		Point max = Point(0.0);
		for (auto v_it = mesh->vertices_sbegin(); v_it != mesh->vertices_end(); ++v_it) {
			auto p = mesh->point(v_it);
			min[0] = std::min(p[0], min[0]);
			min[1] = std::min(p[1], min[1]);
			min[2] = std::min(p[2], min[2]);

			max[0] = std::max(p[0], max[0]);
			max[1] = std::max(p[1], max[1]);
			max[2] = std::max(p[2], max[2]);
		}

		for (auto v_it = mesh->vertices_sbegin(); v_it != mesh->vertices_end(); ++v_it) {
			auto p = mesh->point(v_it);
			TexCoord2D tcZero(p[0] / (max[0] - min[0]), p[1] / (max[1] - min[1]));
			mesh->set_texcoord2D(v_it, tcZero);
		}

		emit updatedObject(meshObj->id(), UPDATE_ALL);
	}
}
