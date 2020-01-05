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

		auto visited = OpenMesh::makeTemporaryProperty<VertexHandle, bool>(*mesh);

		if (!mesh->has_vertex_texcoords2D()) {
			mesh->request_vertex_texcoords2D();
		}

		for (auto f_it = mesh->faces_sbegin(); f_it != mesh->faces_end(); ++f_it) {

			// TODO shorten this method
			auto fv_it = mesh->fv_iter(*f_it);
			auto vh0 = *fv_it++;
			auto vh1 = *fv_it++;
			auto vh2 = *fv_it;

			// Fill all three if they are empty
			if (!visited[vh0] && !visited[vh1] && !visited[vh2]) {

				TexCoord2D tcZero(0.0, 0.0);
				TexCoord2D tcZDir(1.0, 0.0);
				mesh->set_texcoord2D(vh0, tcZero);

				auto vecV01 = mesh->point(vh1) - mesh->point(vh0);
				auto vecV01_n = vecV01;
				auto vecV02 = mesh->point(vh2) - mesh->point(vh0);
				auto vecV02_n = vecV02;
				auto angle = acos(dot(normalize(vecV01_n), normalize(vecV02_n)));

				mesh->set_texcoord2D(vh1, tcZDir * betri::length(vecV01));
				auto tc = TexCoord2D(
					cos(angle) * tcZDir[0] - sin(angle) * tcZDir[1],
					sin(angle) * tcZDir[0] + cos(angle) * tcZDir[1]
				) * betri::length(vecV02);
				mesh->set_texcoord2D(vh2, tc);

				visited[vh0] = true;
				visited[vh1] = true;
				visited[vh2] = true;
			}
			// Fill two if they are empty
			else if (visited[vh0] ^ visited[vh1] ^ visited[vh2]) {

				continue;
			}
			// Fill one if it is empty
			else if (!visited[vh0] ^ !visited[vh1] ^ !visited[vh2]) {

				VertexHandle first;
				VertexHandle second;
				VertexHandle third;

				if (!visited[vh0]) {
					first = vh1;
					second = vh2;
					third = vh0;
				} else if (!visited[vh1]) {
					first = vh2;
					second = vh0;
					third = vh1;
				} else if (!visited[vh2]) {
					first = vh0;
					second = vh1;
					third = vh2;
				}

				TexCoord2D tcSecond = mesh->texcoord2D(second) - mesh->texcoord2D(first);

				auto vecV01 = mesh->point(second) - mesh->point(first);
				auto vecV01_n = vecV01;
				auto vecV02 = mesh->point(third) - mesh->point(first);
				auto vecV02_n = vecV02;
				auto angle = acos(dot(normalize(vecV01_n), normalize(vecV02_n)));

				TexCoord2D tc = TexCoord2D(
					cos(angle) * tcSecond[0] - sin(angle) * tcSecond[1],
					sin(angle) * tcSecond[0] + cos(angle) * tcSecond[1]
				) * betri::length(vecV02) + mesh->texcoord2D(first);

				mesh->set_texcoord2D(third, tc);

				visited[third] = true;
			}
		}

		emit updatedObject(meshObj->id(), UPDATE_ALL);
	}
}
