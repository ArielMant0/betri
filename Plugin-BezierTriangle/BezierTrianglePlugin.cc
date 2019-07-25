#include "BezierTrianglePlugin.hh"
#include "BezierMeshObject.hh"

#include <qpushbutton.h>
#include <qgridlayout.h>

#include "OpenFlipper/BasePlugin/PluginFunctions.hh"

void BezierTrianglePlugin::pluginsInitialized()
{
	addDataType("BezierTriangleMesh", tr("BezierTriangleMesh"));
}

void BezierTrianglePlugin::initializePlugin()
{
	m_tool = new QWidget();
	QIcon *toolIcon = new QIcon(
		OpenFlipper::Options::iconDirStr() +
		OpenFlipper::Options::dirSeparator() +
		"beziertriangle.png"
	);
	QPushButton *loadButton = new QPushButton(tr("Load"));
	QPushButton *decimateButton = new QPushButton(tr("Decimate"));

	QGridLayout *grid = new QGridLayout();
	grid->addWidget(loadButton, 0, 0);
	grid->addWidget(decimateButton, 1, 0);
	m_tool->setLayout(grid);

	connect(loadButton, SIGNAL(clicked()), this, SLOT(loadMesh()));

    emit addToolbox(tr("BezierTriangle"), m_tool, toolIcon);
}

void BezierTrianglePlugin::loadMesh()
{
	PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS, DATA_TRIANGLE_MESH);
	if (o_it != PluginFunctions::objectsEnd()) {
		TriMesh *mesh = dynamic_cast<TriMeshObject*>(*o_it)->mesh();
		createBezierMesh(mesh);
		emit log(LOGINFO, "Created a bezier triangle mesh");
		BezierMeshObject obj(DATA_BEZIER_TRIANGLE_MESH);
		emit log(LOGINFO, "Created a bezier mesh object");
	}
}

void BezierTrianglePlugin::createBezierMesh(TriMesh *mesh)
{
	m_mesh = BezierTMesh(*mesh);
}

//void render(ACG::GLState * _glState, Viewer::ViewerProperties & _properties)
//{
//	ACG::SceneGraph::BaseNode* root = PluginFunctions::getSceneGraphRootNode();
//
//	if (root) {
//		// do stuff
//	}
//}

