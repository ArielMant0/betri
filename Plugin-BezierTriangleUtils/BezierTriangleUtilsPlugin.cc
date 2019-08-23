#include "BezierTriangleUtilsPlugin.hh"

#include <qpushbutton.h>
#include <qgridlayout.h>
// TODO new
#include <qgroupbox.h>
#include <qcombobox.h>
#include <qlabel.h>
#include <qspinbox.h>
#include <qcheckbox.h>

//#include <OpenFlipper/common/UpdateType.hh>
#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <OpenFlipper/common/GlobalOptions.hh>

#include "OpenFlipper/BasePlugin/PluginFunctions.hh"

#include "BezierTriangleUtils.hh"

using namespace betri;

void BezierTriangleUtilsPlugin::initializePlugin()
{
	m_tool = new QWidget();
	QIcon *toolIcon = new QIcon(
		OpenFlipper::Options::iconDirStr() +
		OpenFlipper::Options::dirSeparator() +
		"btutils.png"
	);

	///////////////////////////////////////////////////////////////////////////
	// Voronoi meshing group
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *voronoiGroup = new QGroupBox(tr("Voronoi Meshing"));

	QPushButton *voronoiButton = new QPushButton(tr("Do it"));
	connect(voronoiButton, SIGNAL(clicked()), this, SLOT(callVoronoi()));

	QGridLayout *voronoiLayout = new QGridLayout;
	voronoiLayout->addWidget(voronoiButton, 0, 0);
	voronoiGroup->setLayout(voronoiLayout);

	///////////////////////////////////////////////////////////////////////////
	// Tesselation group
	// https://doc.qt.io/qt-5/qtwidgets-widgets-lineedits-example.html
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *tessGroup = new QGroupBox(tr("Tessellation"));

	// Create Drop down menu
	QLabel *tessMLabel = new QLabel(tr("Mode:"));
	QComboBox *tessComboBox = new QComboBox;
	tessComboBox->addItem(tr("CPU"));
	tessComboBox->addItem(tr("GPU"));
	tessComboBox->addItem(tr("Raytracing"));

	// TODO this was from the tutorial
	//connect(tessComboBox, QOverload<int>::of(&QComboBox::activated),
	//	this, &BezierTriangleUtilsPlugin::echoChanged);

	// set the connection to the setTessType
	// TODO warum ist das hier anders als bei der tessspinbox
	connect(tessComboBox, QOverload<int>::of(&QComboBox::activated),
		this, &BezierTriangleUtilsPlugin::setTessType);

	// Create the gui for tessamount selection
	QLabel *tessALabel = new QLabel(tr("TesselationAmount:"));
	QSpinBox *tessSpinBox = new QSpinBox;
	QSlider *tessSlider = new QSlider(Qt::Horizontal);
	tessSpinBox->setRange(0, 10);
	tessSlider->setRange(0, 10);

	connect(tessSpinBox, SIGNAL(valueChanged(int)), tessSlider, SLOT(setValue(int)));
	connect(tessSlider, SIGNAL(valueChanged(int)), tessSpinBox, SLOT(setValue(int)));
	connect(tessSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setTessAmount(int)));

	tessSpinBox->setValue(1);

	QCheckBox *adaptCBox = new QCheckBox("Adaptive Tesselation");

	// TODO from the turtorial
	echoLineEdit = new QLineEdit;
	echoLineEdit->setPlaceholderText("Placeholder Text");
	echoLineEdit->setFocus();

	QGridLayout *tessLayout = new QGridLayout;
	tessLayout->addWidget(tessMLabel, 0, 0);
	tessLayout->addWidget(tessComboBox, 0, 1);
	tessLayout->addWidget(tessALabel, 1, 0);
	tessLayout->addWidget(tessSpinBox, 1, 1);
	tessLayout->addWidget(tessSlider, 2, 0);
	tessLayout->addWidget(adaptCBox, 3, 0);
	tessLayout->addWidget(echoLineEdit, 4, 0, 1, 2);
	tessGroup->setLayout(tessLayout);

	///////////////////////////////////////////////////////////////////////////
	// Visualisation group
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *visGroup = new QGroupBox(tr("Visualisation"));

	QLabel *visLabel1 = new QLabel(tr("Mode:"));
	QComboBox *visComboBox = new QComboBox;
	visComboBox->addItem(tr("TODO"));
	visComboBox->addItem(tr("TODO"));
	visComboBox->addItem(tr("TODO"));

	connect(visComboBox, QOverload<int>::of(&QComboBox::activated),
		this, &BezierTriangleUtilsPlugin::echoChanged);

	QLabel *visLabel2 = new QLabel(tr("Controll Point size:"));
	QSpinBox *spinBox2 = new QSpinBox;
	QSlider *slider2 = new QSlider(Qt::Horizontal);
	spinBox2->setRange(0, 10);
	slider2->setRange(0, 10);

	connect(spinBox2, SIGNAL(valueChanged(int)), slider2, SLOT(setValue(int)));
	connect(slider2, SIGNAL(valueChanged(int)), spinBox2, SLOT(setValue(int)));
	spinBox2->setValue(1);

	QCheckBox *controlNet = new QCheckBox("Show Controllnet");

	//emit log(LOGINFO, tr("Option 0 is %1").arg(BezierOptionMng::instance().option(BezierOption::TESSELLATION_AMOUNT)));

	QGridLayout *visLayout = new QGridLayout;
	visLayout->addWidget(visLabel1, 0, 0);
	visLayout->addWidget(visComboBox, 0, 1);
	visLayout->addWidget(visLabel2, 1, 0);
	visLayout->addWidget(spinBox2, 1, 1);
	visLayout->addWidget(slider2, 2, 0);
	visLayout->addWidget(controlNet, 3, 0);
	visGroup->setLayout(visLayout);

	///////////////////////////////////////////////////////////////////////////
	// Add all Elements
	///////////////////////////////////////////////////////////////////////////
	QGridLayout *grid = new QGridLayout();
	grid->addWidget(voronoiGroup, 0, 0);
	grid->addWidget(tessGroup, 1, 0);
	grid->addWidget(visGroup, 2, 0);
	m_tool->setLayout(grid);

    emit addToolbox(tr("Bezier Triangle Utils"), m_tool, toolIcon);
}

void BezierTriangleUtilsPlugin::convertMesh()
{
	PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	if (o_it != PluginFunctions::objectsEnd()) {
		BezierTMesh *mesh = dynamic_cast<BTMeshObject*>(*o_it)->mesh();
		emit log(LOGINFO, "Nothing happend, wow!");
	}
}

void BezierTriangleUtilsPlugin::tessellateMesh()
{
	emit log(LOGINFO, "Tessellated Bezier Triangles!");
}

void BezierTriangleUtilsPlugin::echoChanged(int index)
{
	switch (index) {
		case 0:
			echoLineEdit->setEchoMode(QLineEdit::Normal);
			break;
		case 1:
			echoLineEdit->setEchoMode(QLineEdit::Password);
			break;
		case 2:
			echoLineEdit->setEchoMode(QLineEdit::PasswordEchoOnEdit);
			break;
		case 3:
			echoLineEdit->setEchoMode(QLineEdit::NoEcho);
	}
}

void BezierTriangleUtilsPlugin::setTessAmount(int value)
{
	PluginFunctions::betriOption(BezierOption::TESSELLATION_AMOUNT, value);
	emit log(LOGINFO, tr("set tessellation amount to %1").arg(value));
	PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleUtilsPlugin::setTessType(int value)
{
	PluginFunctions::betriOption(BezierOption::TESSELLATION_TYPE, value);
	emit log(LOGINFO, tr("set tessellation type to %1").arg(value));
}

void BezierTriangleUtilsPlugin::callVoronoi()
{
	PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	if (o_it != PluginFunctions::objectsEnd()) {
		PluginFunctions::setDrawMode(ACG::SceneGraph::DrawModes::SOLID_FACES_COLORED);

		BTMeshObject *meshObj = dynamic_cast<BTMeshObject*>(*o_it);
		BezierTMesh *mesh = meshObj->mesh();
		const auto size = mesh->n_faces() * 0.5;
		betri::voronoi(*mesh, size);
		emit log(LOGINFO, "Performed Voronoi Partition!");
		emit updatedObject(meshObj->id(), UPDATE_ALL);

		// DEBUG //
		/*using VH = BezierTMesh::VertexHandle;
		using ID = unsigned int;

		std::vector<ID> regions;
		regions.reserve(size);
		for (auto i = 0; i < size; ++i) {
			regions.push_back(0);
		}
		auto id = OpenMesh::getProperty<VH, ID>(*mesh, "region");
		for (const auto &vh : mesh->vertices()) {
			regions[id[vh]]++;
		}
		for (ID i = 0; i < regions.size(); ++i) {
			emit log(LOGINFO, tr("region %1 contains %2 vertices").arg(i).arg(regions[i]));
		}*/
	}
}
