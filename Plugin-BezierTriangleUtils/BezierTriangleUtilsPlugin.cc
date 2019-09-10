#include "BezierTriangleUtilsPlugin.hh"

#include <qpushbutton.h>
#include <qgridlayout.h>
// TODO new
#include <qgroupbox.h>
#include <qcombobox.h>
#include <qlabel.h>
#include <qspinbox.h>
#include <qcheckbox.h>

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

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
	QPushButton *loadButton = new QPushButton(tr("Add Bezier Triangles"));
	QPushButton *decimateButton = new QPushButton(tr("Decimate"));

	connect(loadButton, SIGNAL(clicked()), this, SLOT(convertMesh())); 

	///////////////////////////////////////////////////////////////////////////
	// Tesselation group
	// https://doc.qt.io/qt-5/qtwidgets-widgets-lineedits-example.html
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *echoGroup = new QGroupBox(tr("Tessellation"));

	QLabel *echoLabel = new QLabel(tr("Mode:"));
	QComboBox *echoComboBox = new QComboBox;
	echoComboBox->addItem(tr("CPU"));
	echoComboBox->addItem(tr("GPU"));
	echoComboBox->addItem(tr("Raytracing"));

	connect(echoComboBox, QOverload<int>::of(&QComboBox::activated),
		this, &BezierTriangleUtilsPlugin::echoChanged);

	connect(echoComboBox, QOverload<int>::of(&QComboBox::activated), this, &BezierTriangleUtilsPlugin::setTessType);

	QLabel *tessLabel = new QLabel(tr("TesselationAmount:"));
	QSpinBox *spinBox = new QSpinBox;
	QSlider *slider = new QSlider(Qt::Horizontal);
	spinBox->setRange(0, 10);
	slider->setRange(0, 10);

	connect(spinBox, SIGNAL(valueChanged(int)), slider, SLOT(setValue(int)));
	connect(slider, SIGNAL(valueChanged(int)), spinBox, SLOT(setValue(int)));
	connect(spinBox, SIGNAL(valueChanged(int)), this, SLOT(setTessAmount(int)));

	spinBox->setValue(1);

	//betri::tesselationAmount_++;
	//int tesselationAmount_ = 0;
	//int tesselationType_ = 0;
	QCheckBox *adapt = new QCheckBox("Adaptive Tesselation");

	echoLineEdit = new QLineEdit;
	echoLineEdit->setPlaceholderText("Placeholder Text");
	echoLineEdit->setFocus();

	QGridLayout *echoLayout = new QGridLayout;
	echoLayout->addWidget(echoLabel, 0, 0);
	echoLayout->addWidget(echoComboBox, 0, 1);
	echoLayout->addWidget(tessLabel, 1, 0);
	echoLayout->addWidget(spinBox, 1, 1);
	echoLayout->addWidget(slider, 2, 0);
	echoLayout->addWidget(adapt, 3, 0);
	echoLayout->addWidget(echoLineEdit, 4, 0, 1, 2);
	echoGroup->setLayout(echoLayout);

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
	grid->addWidget(loadButton, 0, 0);
	grid->addWidget(decimateButton, 1, 0);
	grid->addWidget(echoGroup, 2, 0);
	grid->addWidget(visGroup, 3, 0);
	m_tool->setLayout(grid);

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
