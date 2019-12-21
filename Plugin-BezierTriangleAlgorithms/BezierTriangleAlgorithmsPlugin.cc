#include "BezierTriangleAlgorithmsPlugin.hh"

#include <qpushbutton.h>
#include <qgridlayout.h>
// TODO new
#include <qgroupbox.h>
#include <qcombobox.h>
#include <qlabel.h>
#include <qinputdialog.h>

#include <OpenFlipper/common/GlobalOptions.hh>

#include "OpenFlipper/BasePlugin/PluginFunctions.hh"

#include "BezierTriangleAlgorithms.hh"

using namespace betri;

void BezierTriangleAlgorithmsPlugin::initializePlugin()
{

	m_tool = new QWidget();
	QIcon *toolIcon = new QIcon(
		OpenFlipper::Options::iconDirStr() +
		OpenFlipper::Options::dirSeparator() +
		"btalgorithms.png"
	);

	// https://doc.qt.io/qt-5/qtwidgets-widgets-lineedits-example.html
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
	QPushButton *smoothButton = new QPushButton(tr("Smooth"));
	connect(smoothButton, SIGNAL(clicked()), this, SLOT(callSmooth()));

	m_voronoiSteps.push_back(patitionButton);
	m_voronoiSteps.push_back(dualStepButton);
	m_voronoiSteps.push_back(dualButton);
	m_voronoiSteps.push_back(fittingButton);

	m_colors[0] = new QCheckBox(tr("use colors"));
	m_colors[0]->setChecked(true); // default

	QGridLayout *voronoiLayout = new QGridLayout;
	voronoiLayout->addWidget(voronoiButton, 0, 0, 1, 3);
	voronoiLayout->addWidget(patitionButton, 1, 0, 2, 1);
	voronoiLayout->addWidget(dualStepButton, 1, 1, 1, 1);
	voronoiLayout->addWidget(dualButton, 2, 1, 1, 1);
	voronoiLayout->addWidget(fittingButton, 1, 2, 2, 1);
	voronoiLayout->addWidget(smoothButton, 3, 0, 1, 3);
	voronoiLayout->addWidget(m_colors[0], 4, 0);
	voronoiGroup->setLayout(voronoiLayout);

	///////////////////////////////////////////////////////////////////////////
	// Decimation group
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *deciGroup = new QGroupBox(tr("Decimation"));

	QPushButton *deciInitButton = new QPushButton(tr("Intialize Decimation"));
	connect(deciInitButton, SIGNAL(clicked()), this, SLOT(callDecimationInit()));
	QPushButton *deciButton = new QPushButton(tr("Full Decimation"));
	connect(deciButton, SIGNAL(clicked()), this, SLOT(callDecimation()));
	QPushButton *deciStepButton = new QPushButton(tr("Step Decimation"));
	connect(deciStepButton, SIGNAL(clicked()), this, SLOT(callDecimationStep()));

	m_flags[1] = new QCheckBox(tr("interpolate"));
	m_colors[1] = new QCheckBox(tr("use colors"));

	QGridLayout *deciLayout = new QGridLayout;
	deciLayout->addWidget(deciInitButton, 0, 0);
	deciLayout->addWidget(deciButton, 1, 0);
	deciLayout->addWidget(deciStepButton, 2, 0);
	deciLayout->addWidget(m_colors[1], 3, 0);
	deciLayout->addWidget(m_flags[1], 4, 0);
	deciGroup->setLayout(deciLayout);

	///////////////////////////////////////////////////////////////////////////
	// Tests
	///////////////////////////////////////////////////////////////////////////
	QPushButton *testButton = new QPushButton(tr("Start Test"));
	connect(testButton, SIGNAL(clicked()), this, SLOT(testAlgorithm()));

	///////////////////////////////////////////////////////////////////////////
	// Add all Elements
	///////////////////////////////////////////////////////////////////////////

	QGridLayout *grid = new QGridLayout();
	grid->addWidget(voronoiGroup, 0, 0);
	grid->addWidget(deciGroup, 1, 0);
	grid->addWidget(testButton, 2, 0);
	m_tool->setLayout(grid);

    emit addToolbox(tr("Bezier Triangle Algorithms"), m_tool, toolIcon);
}

void BezierTriangleAlgorithmsPlugin::callVoronoi()
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

		BTMeshObject *meshObj = dynamic_cast<BTMeshObject*>(*o_it);

		bool ok;
		int minValue = meshObj->mesh()->n_faces()*0.01;
		int seedCount = QInputDialog::getInt(
			m_tool,
			"Voronoi Meshing",
			"Please enter the minimum number of seeds: ",
			// value, min value
			minValue, 0,
			// max value, steps
			meshObj->mesh()->n_faces(), 1, &ok
		);

		if (ok) {
			betri::voronoiInit(*o_it, ctrl_obj, seedCount, true);
			betri::voronoiRemesh(*o_it, ctrl_obj);

			BezierTMesh *cMesh = ctrlMeshObj->mesh();

			emit log(LOGINFO, "# --------------------------- #");
			emit log(LOGINFO, "# Performed Voronoi Remeshing!");
			emit log(LOGINFO, tr("# Vertices: %1").arg(cMesh->n_vertices()));
			emit log(LOGINFO, tr("# Edges: %1").arg(cMesh->n_edges()));
			emit log(LOGINFO, tr("# Faces: %1").arg(cMesh->n_faces()));
			emit log(LOGINFO, "# --------------------------- #");

			emit updatedObject(meshObj->id(), UPDATE_ALL);
			emit updatedObject(ctrl_id, UPDATE_ALL);

			meshObj->hide();
			cMesh->setRenderable();
			ctrlMeshObj->show();
		}
	}
}

void BezierTriangleAlgorithmsPlugin::callDualStep()
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

		// only show control mesh obj if its complete
		if (done) {
			emit log(LOGINFO, "Finished Dualizing!");

			emit updatedObject(meshObj->id(), UPDATE_ALL);
			emit updatedObject(ctrlMeshObj->id(), UPDATE_ALL);

			// TODO: does not work for some reason
			ctrlMeshObj->mesh()->setRenderable();
			ctrlMeshObj->show();

			//m_voronoiSteps[1]->setDisabled(true);
			//m_voronoiSteps[2]->setDisabled(true);
		} else {
			emit updatedObject(meshObj->id(), UPDATE_ALL);
		}
	}
}

void BezierTriangleAlgorithmsPlugin::callDual()
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

		emit updatedObject(meshObj->id(), UPDATE_ALL);
		emit updatedObject(ctrlMeshObj->id(), UPDATE_ALL);

		// TODO: does not work for some reason
		ctrlMeshObj->mesh()->setRenderable();
		ctrlMeshObj->show();

		//m_voronoiSteps[1]->setDisabled(true);
		//m_voronoiSteps[2]->setDisabled(true);
	}
}

void BezierTriangleAlgorithmsPlugin::callFitting()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);
		BTMeshObject *ctrlMeshObj = PluginFunctions::btMeshObject(ctrl_obj);

		BezierTMesh *cMesh = ctrlMeshObj->mesh();

		betri::voronoiFitting(*o_it, ctrl_obj);
		emit log(LOGINFO, "# --------------------------- #");
		emit log(LOGINFO, "# Performed Fitting!");
		emit log(LOGINFO, tr("# Vertices: %1").arg(cMesh->n_vertices()));
		emit log(LOGINFO, tr("# Edges: %1").arg(cMesh->n_edges()));
		emit log(LOGINFO, tr("# Faces: %1").arg(cMesh->n_faces()));
		emit log(LOGINFO, "# --------------------------- #");

		cMesh->setRenderable();

		emit updatedObject(meshObj->id(), UPDATE_ALL);
		emit updatedObject(ctrlMeshObj->id(), UPDATE_ALL);

		meshObj->hide();
		ctrlMeshObj->show();
	}
	for (auto button : m_voronoiSteps) {
		button->setDisabled(false);
	}

}

void BezierTriangleAlgorithmsPlugin::callSmooth()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);
		BTMeshObject *ctrlMeshObj = PluginFunctions::btMeshObject(ctrl_obj);

		betri::voronoiSmooth(*o_it, ctrl_obj);
		emit log(LOGINFO, "Performed Smoothing!");

		emit updatedObject(ctrlMeshObj->id(), UPDATE_ALL);
	}
}

void BezierTriangleAlgorithmsPlugin::callPartition()
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

		bool ok;
		int minValue = meshObj->mesh()->n_faces()*0.01;
		int seedCount = QInputDialog::getInt(
			m_tool,
			"Voronoi Meshing",
			"Please enter the minimum number of seeds: ",
			// value, min value
			minValue, 0,
			// max value, steps
			meshObj->mesh()->n_faces(), 1, &ok
		);

		if (ok) {
			betri::voronoiInit(*o_it, ctrl_obj, seedCount, true);
			betri::voronoiPartition(*o_it, ctrl_obj);
			emit log(LOGINFO, "Performed Voronoi Partition!");

			//emit updatedObject(meshObj->id(), UPDATE_COLOR);
			emit updatedObject(meshObj->id(), UPDATE_ALL);
		}

		//m_voronoiSteps[0]->setDisabled(true);
	}
}

void BezierTriangleAlgorithmsPlugin::testAlgorithm()
{
	QStringList items;
	items << tr("Voronoi Parametrization");
	items << tr("Voronoi Fitting");
	items << tr("Decimation Parametrization");
	items << tr("Decimation Fitting");

	bool okay;
	QString option = QInputDialog::getItem(
		m_tool,
		"Test Algorithm",
		"Please choose an algorithm to test: ",
		items,
		// current selection, editable
		0, false, &okay
	);

	if (okay && !option.isEmpty()) {
		int ctrl_id;

		emit addEmptyObject(DATA_BEZIER_TRIANGLE_MESH, ctrl_id);
		PluginFunctions::getObject(ctrl_id, ctrl_obj);
		BTMeshObject *ctrlMeshObj = PluginFunctions::btMeshObject(ctrl_obj);
		ctrlMeshObj->target(true);
		ctrlMeshObj->setName("test-mesh");

		std::string item = option.toStdString();
		if (item.compare("Voronoi Parametrization") == 0) {
			betri::test(betri::TestOptions::voronoi_param, ctrlMeshObj->mesh());
		} else if (item.compare("Voronoi Fitting") == 0) {
			betri::test(betri::TestOptions::voronoi_fit, ctrlMeshObj->mesh());
		} else if (item.compare("Decimation Parametrization") == 0) {
			betri::test(betri::TestOptions::decimation_param, ctrlMeshObj->mesh());
		} else if (item.compare("Decimation Fitting") == 0) {
			betri::test(betri::TestOptions::decimation_fit, ctrlMeshObj->mesh());
		}

		emit updatedObject(ctrlMeshObj->id(), UPDATE_ALL);

		ctrlMeshObj->mesh()->setRenderable();
		ctrlMeshObj->show();
	}
}

void BezierTriangleAlgorithmsPlugin::callDecimationInit()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);
		BezierTMesh *mesh = meshObj->mesh();

		bool okay = false;

		int meshId = meshObj->id();
		int target = QInputDialog::getInt(
			m_tool,
			"Decimation Meshing",
			"Please enter target complexity: ",
			// value, min value
			mesh->n_vertices(), 10,
			// max value, steps
			mesh->n_vertices(), 1, &okay
		);

		if (okay) {
			betri::decimationInit(meshObj, target, m_colors[1]->isChecked());

			emit updatedObject(meshObj->id(), UPDATE_ALL);
		}
	}
}

void BezierTriangleAlgorithmsPlugin::callDecimation()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);
		BezierTMesh *mesh = meshObj->mesh();

		betri::decimation(meshObj, false, m_flags[1]->isChecked());
		emit log(LOGINFO, "# --------------------------- #");
		emit log(LOGINFO, "# Performed Decimation (DONE)!");
		emit log(LOGINFO, tr("# Vertices: %1").arg(mesh->n_vertices()));
		emit log(LOGINFO, tr("# Edges: %1").arg(mesh->n_edges()));
		emit log(LOGINFO, tr("# Faces: %1").arg(mesh->n_faces()));
		emit log(LOGINFO, "# --------------------------- #");

		emit updatedObject(meshObj->id(), UPDATE_ALL);
	}
}

// TODO: not really useful right now and also does not work
void BezierTriangleAlgorithmsPlugin::callDecimationStep()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);

		const bool done = betri::decimation(
			meshObj,
			true,
			m_flags[1]->isChecked()
		);

		if (done) {
			BezierTMesh *mesh = meshObj->mesh();

			emit log(LOGINFO, "# --------------------------- #");
			emit log(LOGINFO, "# Performed Decimation (DONE)!");
			emit log(LOGINFO, tr("# Vertices: %1").arg(mesh->n_vertices()));
			emit log(LOGINFO, tr("# Edges: %1").arg(mesh->n_edges()));
			emit log(LOGINFO, tr("# Faces: %1").arg(mesh->n_faces()));
			emit log(LOGINFO, "# --------------------------- #");

		} else {
			emit log(LOGINFO, "Performed Decimation Step!");
		}

		emit updatedObject(meshObj->id(), UPDATE_ALL);
	}
}

