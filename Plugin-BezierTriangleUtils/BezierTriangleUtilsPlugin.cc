#include "BezierTriangleUtilsPlugin.hh"

#include <qpushbutton.h>
#include <qgridlayout.h>
// TODO new
#include <qgroupbox.h>
#include <qcombobox.h>
#include <qlabel.h>
#include <qspinbox.h>
#include <qcheckbox.h>
#include <qinputdialog.h>

//#include <OpenFlipper/common/UpdateType.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <OpenFlipper/common/GlobalOptions.hh>
#include <OpenFlipper/common/RendererInfo.hh>

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

	QPushButton *voronoiButton = new QPushButton(tr("Voronoi Meshing"));
	connect(voronoiButton, SIGNAL(clicked()), this, SLOT(callVoronoi()));

	QPushButton *patitionButton = new QPushButton(tr("Part."));
	connect(patitionButton, SIGNAL(clicked()), this, SLOT(callPartition()));
	QPushButton *dualStepButton = new QPushButton(tr("Dual Step"));
	connect(dualStepButton, SIGNAL(clicked()), this, SLOT(callDualStep()));
	QPushButton *dualButton = new QPushButton(tr("Dual"));
	connect(dualButton, SIGNAL(clicked()), this, SLOT(callDual()));
	QPushButton *fittingButton = new QPushButton(tr("Fit"));
	connect(fittingButton, SIGNAL(clicked()), this, SLOT(callFitting()));

	QPushButton *testFitButton = new QPushButton(tr("Test Fitting"));
	connect(testFitButton, SIGNAL(clicked()), this, SLOT(testFitting()));
	QPushButton *testParamButton = new QPushButton(tr("Test Parametrization"));
	connect(testParamButton, SIGNAL(clicked()), this, SLOT(testParametrization()));

	m_voronoiSteps.push_back(patitionButton);
	m_voronoiSteps.push_back(dualStepButton);
	m_voronoiSteps.push_back(dualButton);
	m_voronoiSteps.push_back(fittingButton);

	QGridLayout *voronoiLayout = new QGridLayout;
	voronoiLayout->addWidget(voronoiButton, 0, 0, 1, 4);
	voronoiLayout->addWidget(patitionButton, 1, 0);
	voronoiLayout->addWidget(dualStepButton, 1, 1);
	voronoiLayout->addWidget(dualButton, 1, 2);
	voronoiLayout->addWidget(fittingButton, 1, 3);
	voronoiLayout->addWidget(testFitButton, 2, 0, 1, 4);
	voronoiLayout->addWidget(testParamButton, 3, 0, 1, 4);
	voronoiGroup->setLayout(voronoiLayout);

	///////////////////////////////////////////////////////////////////////////
	// Decimation group
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *deciGroup = new QGroupBox(tr("Decimation"));

	QPushButton *deciButton = new QPushButton(tr("Decimation"));
	connect(deciButton, SIGNAL(clicked()), this, SLOT(callDecimation()));
	QPushButton *deciStepButton = new QPushButton(tr("Decimation Step"));
	connect(deciStepButton, SIGNAL(clicked()), this, SLOT(callDecimationStep()));

	QGridLayout *deciLayout = new QGridLayout;
	deciLayout->addWidget(deciButton, 0, 0);
	deciLayout->addWidget(deciStepButton, 1, 0);
	deciGroup->setLayout(deciLayout);

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
	grid->addWidget(deciGroup, 1, 0);
	grid->addWidget(tessGroup, 2, 0);
	grid->addWidget(visGroup, 3, 0);
	m_tool->setLayout(grid);

    emit addToolbox(tr("Bezier Triangle Utils"), m_tool, toolIcon);
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

	// TODO: get these names in a more robust manner
	switch (value) {
		default: // CPU
			renderManager().setActive(QString("Default Classical Renderer"), PluginFunctions::activeExaminer());
			break;
		case 1: // GPU
			renderManager().setActive(QString("Alpha_Version_ShaderPipeline"), PluginFunctions::activeExaminer());
			break;
		case 2: // raytracing
			renderManager().setActive(QString("Raytracing_Renderer"), PluginFunctions::activeExaminer());
			break;
	}
	emit log(LOGINFO, tr("set tessellation type to %1").arg(value));
}

void BezierTriangleUtilsPlugin::callVoronoi()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		int ctrl_id;
		BTMeshObject *ctrlMeshObj;

		emit addEmptyObject(DATA_BEZIER_TRIANGLE_MESH, ctrl_id);
		PluginFunctions::getObject(ctrl_id, ctrl_obj);
		ctrlMeshObj = PluginFunctions::btMeshObject(ctrl_obj);
		ctrlMeshObj->setName("control mesh");
		ctrlMeshObj->hide();
		ctrlMeshObj->target(false);

		betri::voronoiInit(*o_it, ctrl_obj, true);
		betri::voronoiRemesh(*o_it, ctrl_obj);
		emit log(LOGINFO, "Performed Voronoi Remeshing!");

		BTMeshObject *meshObj = dynamic_cast<BTMeshObject*>(*o_it);
		emit updatedObject(meshObj->id(), UPDATE_ALL);
		emit updatedObject(ctrl_id, UPDATE_ALL);

		ctrlMeshObj->mesh()->setRenderable();
		ctrlMeshObj->show();
	}
}

void BezierTriangleUtilsPlugin::callDualStep()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);
		BTMeshObject *ctrlMeshObj = PluginFunctions::btMeshObject(ctrl_obj);

		bool done = betri::voronoiDual(*o_it, ctrl_obj, true);
		emit log(LOGINFO, "Performed Dualizing Step!");

		emit updatedObject(meshObj->id(), UPDATE_COLOR);

		// only show control mesh obj if its complete
		if (done) {
			emit log(LOGINFO, "Finished Dualizing!");

			emit updatedObject(ctrlMeshObj->id(), UPDATE_ALL);

			// TODO: does not work for some reason
			ctrlMeshObj->mesh()->setRenderable();
			ctrlMeshObj->show();

			//m_voronoiSteps[1]->setDisabled(true);
			//m_voronoiSteps[2]->setDisabled(true);
		}
	}
}

void BezierTriangleUtilsPlugin::callDual()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);
		BTMeshObject *ctrlMeshObj = PluginFunctions::btMeshObject(ctrl_obj);

		betri::voronoiDual(*o_it, ctrl_obj, false);
		emit log(LOGINFO, "Performed Dualizing!");

		emit updatedObject(meshObj->id(), UPDATE_COLOR);
		emit updatedObject(ctrlMeshObj->id(), UPDATE_ALL);

		// TODO: does not work for some reason
		ctrlMeshObj->mesh()->setRenderable();
		ctrlMeshObj->show();

		//m_voronoiSteps[1]->setDisabled(true);
		//m_voronoiSteps[2]->setDisabled(true);
	}
}

void BezierTriangleUtilsPlugin::callFitting()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);
		BTMeshObject *ctrlMeshObj = PluginFunctions::btMeshObject(ctrl_obj);

		betri::voronoiFitting(*o_it, ctrl_obj);
		emit log(LOGINFO, "Performed Fitting!");

		emit updatedObject(meshObj->id(), UPDATE_ALL);
		emit updatedObject(ctrlMeshObj->id(), UPDATE_ALL);

		ctrlMeshObj->mesh()->setRenderable();
		ctrlMeshObj->show();
	}
	for (auto button : m_voronoiSteps) {
		button->setDisabled(false);
	}

}

void BezierTriangleUtilsPlugin::callPartition()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		int ctrl_id;
		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);

		emit addEmptyObject(DATA_BEZIER_TRIANGLE_MESH, ctrl_id);
		PluginFunctions::getObject(ctrl_id, ctrl_obj);
		BTMeshObject *ctrlMeshObj = PluginFunctions::btMeshObject(ctrl_obj);

		ctrlMeshObj->setName(meshObj->name() + "_ctrl");
		ctrlMeshObj->hide();
		ctrlMeshObj->target(false);

		betri::voronoiInit(*o_it, ctrl_obj, true);
		betri::voronoiPartition(*o_it, ctrl_obj);
		emit log(LOGINFO, "Performed Voronoi Partition!");

		emit updatedObject(meshObj->id(), UPDATE_COLOR);

		//m_voronoiSteps[0]->setDisabled(true);
	}
}

void BezierTriangleUtilsPlugin::testFitting()
{
	int ctrl_id;

	emit addEmptyObject(DATA_BEZIER_TRIANGLE_MESH, ctrl_id);
	PluginFunctions::getObject(ctrl_id, ctrl_obj);
	BTMeshObject *ctrlMeshObj = PluginFunctions::btMeshObject(ctrl_obj);

	ctrlMeshObj->setName("fitting-test");
	ctrlMeshObj->target(true);

	betri::test(betri::TestOptions::fitting, ctrlMeshObj->mesh());

	emit updatedObject(ctrlMeshObj->id(), UPDATE_ALL);

	ctrlMeshObj->mesh()->setRenderable();
	ctrlMeshObj->show();
}

void BezierTriangleUtilsPlugin::testParametrization()
{
	int ctrl_id;

	emit addEmptyObject(DATA_BEZIER_TRIANGLE_MESH, ctrl_id);
	PluginFunctions::getObject(ctrl_id, ctrl_obj);
	BTMeshObject *ctrlMeshObj = PluginFunctions::btMeshObject(ctrl_obj);

	ctrlMeshObj->setName("parametrization-test");
	ctrlMeshObj->target(true);

	betri::test(betri::TestOptions::parametrization, ctrlMeshObj->mesh());

	emit updatedObject(ctrlMeshObj->id(), UPDATE_ALL);

	ctrlMeshObj->mesh()->setRenderable();
	ctrlMeshObj->show();
}

void BezierTriangleUtilsPlugin::callDecimation()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);
		BezierTMesh *mesh = meshObj->mesh();

		int complexity = QInputDialog::getInt(
			m_tool,
			"Decimation Meshing",
			"Please enter target complexity: ",
			// value, min value
			mesh->n_vertices(), 0,
			// max value, steps
			mesh->n_vertices(), 1
		);

		betri::decimation(meshObj, complexity, false);
		emit log(LOGINFO, "Performed Decimation (DONE)!");

		emit updatedObject(meshObj->id(), UPDATE_ALL);
	}
}

// TODO: not really useful right now and also does not work
void BezierTriangleUtilsPlugin::callDecimationStep()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);

		if (m_target == 0) {
			BezierTMesh *mesh = meshObj->mesh();

			int complexity = QInputDialog::getInt(
				m_tool,
				"Decimation Meshing",
				"Please enter target complexity: ",
				// value, min value
				mesh->n_vertices(), 0,
				// max value, steps
				mesh->n_vertices(), 1
			);
			m_target = complexity;
		}

		const bool done = betri::decimation(meshObj, m_target, true);
		if (done) {
			emit log(LOGINFO, "Performed Decimation Step (DONE)!");
		} else {
			emit log(LOGINFO, "Performed Decimation Step!");
		}
	}
}

