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

	connect(applyTessTri, QOverload<>::of(&QPushButton::pressed),
		this, [&]() {
		applyTessellation(true);
	});
	connect(applyTess, QOverload<>::of(&QPushButton::pressed),
		this, [&]() {
		applyTessellation(false);
	});

	///////////////////////////////////////////////////////////////////////////
	// Add all Elements
	///////////////////////////////////////////////////////////////////////////

	QGridLayout *grid = new QGridLayout();
	grid->addWidget(applyTess, 0, 0);
	grid->addWidget(applyTessTri, 1, 0);
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
