#include "BezierTriangleUtilsPlugin.hh"

#include <qpushbutton.h>
#include <qgridlayout.h>

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

#include "OpenFlipper/BasePlugin/PluginFunctions.hh"

#include "BezierTriangleUtils.hh"

void BezierTriangleUtilsPlugin::initializePlugin()
{
	m_tool = new QWidget();
	QIcon *toolIcon = new QIcon(
		OpenFlipper::Options::iconDirStr() +
		OpenFlipper::Options::dirSeparator() +
		"btutils.png"
	);
	QPushButton *loadButton = new QPushButton(tr("Add Bezier Triangles"));
	QPushButton *decimateButton = new QPushButton(tr("Decimate"));

	QGridLayout *grid = new QGridLayout();
	grid->addWidget(loadButton, 0, 0);
	grid->addWidget(decimateButton, 1, 0);
	m_tool->setLayout(grid);

	connect(loadButton, SIGNAL(clicked()), this, SLOT(convertMesh()));

    emit addToolbox(tr("Bezier Triangle Utils"), m_tool, toolIcon);
}

void BezierTriangleUtilsPlugin::convertMesh()
{
	PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	if (o_it != PluginFunctions::objectsEnd()) {
		BezierTMesh *mesh = dynamic_cast<BTMeshObject*>(*o_it)->mesh();
		betri::addBezierTriangles(*mesh);
		emit log(LOGINFO, "Added Bezier Triangles!");
	}
}
