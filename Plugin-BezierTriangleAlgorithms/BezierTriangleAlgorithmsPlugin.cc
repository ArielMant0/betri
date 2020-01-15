#include "BezierTriangleAlgorithmsPlugin.hh"

#include <qpushbutton.h>
#include <qgridlayout.h>

#include <qgroupbox.h>
#include <qlabel.h>
#include <qinputdialog.h>
#include <qerrormessage.h>

#include <OpenFlipper/common/GlobalOptions.hh>
#include <OpenFlipper/Utils/Memory/RAMInfo.hh>

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

	m_tool->setStyleSheet("QPushButton { padding: 5px; } ");
	// TODO: change margin/padding/sth
	//"QComboBox, QSpinBox { margin-left: 10px; }");

	QLabel *degLabel = new QLabel(tr("Target Degree"));
	QSpinBox *degBox = new QSpinBox;
	degBox->setMinimum(1);
	degBox->setMaximum(10);
	degBox->setValue(betri::globalDegree()); // must be before connect
	connect(degBox, SIGNAL(valueChanged(int)), this, SLOT(setTargetDegree(int)));

	///////////////////////////////////////////////////////////////////////////
	// Voronoi meshing group
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *voronoiGroup = new QGroupBox(tr("Voronoi Meshing"));

	QPushButton *voronoiButton = new QPushButton(tr("Voronoi Meshing"));
	connect(voronoiButton, SIGNAL(clicked()), this, SLOT(callVoronoi()));

	QPushButton *patitionButton = new QPushButton(tr("Part."));
	connect(patitionButton, SIGNAL(clicked()), this, SLOT(callPartition()));
	QPushButton *partStepButton = new QPushButton(tr("Part. Step"));
	connect(partStepButton, SIGNAL(clicked()), this, SLOT(callPartitionStep()));
	QPushButton *dualStepButton = new QPushButton(tr("Dual Step"));
	connect(dualStepButton, SIGNAL(clicked()), this, SLOT(callDualStep()));
	QPushButton *dualButton = new QPushButton(tr("Dual"));
	connect(dualButton, SIGNAL(clicked()), this, SLOT(callDual()));
	QPushButton *fittingButton = new QPushButton(tr("Fit"));
	connect(fittingButton, SIGNAL(clicked()), this, SLOT(callFitting()));

	m_voronoiSteps.push_back(patitionButton);
	m_voronoiSteps.push_back(dualStepButton);
	m_voronoiSteps.push_back(dualButton);
	m_voronoiSteps.push_back(fittingButton);

	m_vFlags[0] = new QCheckBox(tr("colors"));
	m_vFlags[0]->setChecked(true); // default
	m_vFlags[1] = new QCheckBox(tr("interpolate"));
	m_vFlags[2] = new QCheckBox(tr("overwrite mesh"));
	m_vFlags[2]->setChecked(true); // default
	m_vFlags[3] = new QCheckBox(tr("split long edges"));
	m_vFlags[3]->setChecked(true); // default

	QLabel *modeLabel = new QLabel(tr("Parametrization Weights"));
	m_vparam = new QComboBox();
	m_vparam->addItem("uniform");
	m_vparam->addItem("cotangent");
	m_vparam->setCurrentIndex(1); // default cotangent

	QLabel *vsolverLabel = new QLabel(tr("Fitting Solver"));
	m_vfit = new QComboBox();
	m_vfit->addItem("normal equations");
	m_vfit->addItem("qr decomposition");
	m_vfit->addItem("adaptive");
	m_vfit->setCurrentIndex(2); // default adaptive

	QLabel *sampleLabelV = new QLabel(tr("Max. Fitting Samples"));
	m_numSamples[0] = new QSpinBox;
	m_numSamples[0]->setMinimum(0);
	m_numSamples[0]->setMaximum(std::numeric_limits<int>::max()/2);
	m_numSamples[0]->setValue(0);

	// set layout
	QGridLayout *voronoiLayout = new QGridLayout;
	voronoiLayout->addWidget(voronoiButton, 0, 0, 1, 3);
	voronoiLayout->addWidget(patitionButton, 1, 0, 1, 1);
	voronoiLayout->addWidget(partStepButton, 2, 0, 1, 1);
	voronoiLayout->addWidget(dualButton, 1, 1, 1, 1);
	voronoiLayout->addWidget(dualStepButton, 2, 1, 1, 1);
	voronoiLayout->addWidget(fittingButton, 1, 2, 2, 1);
	voronoiLayout->addWidget(m_vFlags[0], 3, 0, 1, 1);
	voronoiLayout->addWidget(m_vFlags[1], 3, 1, 1, 1);
	voronoiLayout->addWidget(m_vFlags[2], 4, 0, 1, 1);
	voronoiLayout->addWidget(m_vFlags[3], 4, 1, 1, 1);
	voronoiLayout->addWidget(modeLabel, 5, 0, 1, 1);
	voronoiLayout->addWidget(m_vparam, 5, 1, 1, 2);
	voronoiLayout->addWidget(vsolverLabel, 6, 0, 1, 1);
	voronoiLayout->addWidget(m_vfit, 6, 1, 1, 2);
	voronoiLayout->addWidget(sampleLabelV, 7, 0, 1, 1);
	voronoiLayout->addWidget(m_numSamples[0], 7, 1, 1, 2);
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

	m_dFlags[0] = new QCheckBox(tr("display error"));
	m_dFlags[1] = new QCheckBox(tr("interpolate"));

	QLabel *dsolverLabel = new QLabel(tr("Fitting Solver"));
	m_dfit = new QComboBox();
	m_dfit->addItem("normal equations");
	m_dfit->addItem("qr decomposition");
	m_dfit->addItem("adaptive");
	m_dfit->setCurrentIndex(2); // default adaptive

	QLabel *sampleLabelD = new QLabel(tr("Fitting Samples"));
	m_numSamples[1] = new QSpinBox;
	m_numSamples[1]->setValue(40);
	m_numSamples[1]->setMinimum(betri::pointsFromDegree(betri::globalDegree()));
	m_numSamples[1]->setMaximum(std::numeric_limits<int>::max()/2);

	QGridLayout *deciLayout = new QGridLayout;
	deciLayout->addWidget(deciInitButton, 0, 0, 1, 2);
	deciLayout->addWidget(deciButton, 1, 0, 1, 1);
	deciLayout->addWidget(deciStepButton, 1, 1, 1, 1);
	deciLayout->addWidget(m_dFlags[0], 2, 0);
	deciLayout->addWidget(m_dFlags[1], 3, 0);
	deciLayout->addWidget(dsolverLabel, 4, 0, 1, 1);
	deciLayout->addWidget(m_dfit, 4, 1, 1, 1);
	deciLayout->addWidget(sampleLabelD, 5, 0, 1, 1);
	deciLayout->addWidget(m_numSamples[1], 5, 1, 1, 1);
	deciGroup->setLayout(deciLayout);

	///////////////////////////////////////////////////////////////////////////
	// Timer
	///////////////////////////////////////////////////////////////////////////
	QCheckBox *useTimer = new QCheckBox(tr("time algorithm"));
	connect(useTimer, QOverload<bool>::of(&QCheckBox::toggled), this, [&](bool use) {
		m_useTimer = use;
	});

	///////////////////////////////////////////////////////////////////////////
	// Add all Elements
	///////////////////////////////////////////////////////////////////////////

	QGridLayout *grid = new QGridLayout();
	grid->addWidget(degLabel, 0, 0, 1, 1);
	grid->addWidget(degBox, 0, 1, 1, 1);
	grid->addWidget(useTimer, 1, 0);
	grid->addWidget(voronoiGroup, 2, 0, 1, 2);
	grid->addWidget(deciGroup, 3, 0, 1, 2);
	m_tool->setLayout(grid);

    emit addToolbox(tr("Bezier Triangle Algorithms"), m_tool, toolIcon);
}

void BezierTriangleAlgorithmsPlugin::setTargetDegree(int degree)
{
	betri::globalDegree(degree);

	// make sure no less than the required number of control
	// points can be set as fitting samples
	m_numSamples[1]->setMinimum((int)betri::pointsFromDegree(degree));
}

bool BezierTriangleAlgorithmsPlugin::applyTargetDegree(BaseObject *meshObj)
{
	BTMeshObject *btmeshObj = dynamic_cast<BTMeshObject*>(meshObj);
	BezierTMesh *m = btmeshObj->mesh();

	if (m->degree() != betri::globalDegree()) {

		m->degree(betri::globalDegree());

		for (BezierTMesh::FaceHandle fh : m->faces()) {
			m->recalculateCPs(fh);
		}

		return true;
	}
	return false;
}

void BezierTriangleAlgorithmsPlugin::callVoronoi()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = dynamic_cast<BTMeshObject*>(*o_it);

		if (applyTargetDegree(*o_it)) {
			emit updatedObject(meshObj->id(), UPDATE_ALL);
		}

		bool ok = true;
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

			int ctrl_id;
			BTMeshObject *ctrlMeshObj;

			emit addEmptyObject(DATA_BEZIER_TRIANGLE_MESH, ctrl_id);
			PluginFunctions::getObject(ctrl_id, ctrl_obj);
			ctrlMeshObj = PluginFunctions::btMeshObject(ctrl_obj);
			ctrlMeshObj->setName("control mesh");
			ctrlMeshObj->hide();
			ctrlMeshObj->target(false);

			betri::voronoiInit(
				*o_it,
				ctrl_obj,
				seedCount,
				m_vFlags[0]->isChecked(),
				m_vFlags[1]->isChecked(),
				m_vFlags[2]->isChecked(),
				m_vFlags[3]->isChecked(),
				m_vparam->currentIndex(),
				m_numSamples[0]->value(),
				m_vfit->currentIndex()
			);

			if (m_useTimer) {
				m_timer.filename(
					meshObj->path().toStdString() +
					"/voronoi-times.txt"
				);
				auto info = betri::voronoiInfo(meshObj, ctrlMeshObj);
				m_timer.start(
					info.name + " " +
					info.vertices + " " +
					info.edges + " " +
					info.faces + " " +
					info.partition
				);
			}

			bool success = betri::voronoiRemesh(*o_it, ctrl_obj);

			if (m_useTimer) {
				auto info = betri::voronoiInfo(meshObj, ctrlMeshObj);
				m_timer.end(
					"\n\tvertices: " + info.vertices +
					"\n\tedges: " + info.edges +
					"\n\tfaces: " + info.faces
				);
			}

			if (success) {
				BezierTMesh *cMesh = ctrlMeshObj->mesh();

				if (m_vFlags[2]->isChecked()) {
					cMesh = meshObj->mesh();
					// remove tmp object
					emit deleteObject(ctrl_id);
					ctrl_obj = nullptr;
				} else {
					cMesh->setRenderable();
					ctrlMeshObj->show();
					meshObj->hide();
				}

				emit updatedObject(meshObj->id(), UPDATE_ALL);

				emit log(LOGINFO, "# --------------------------- #");
				emit log(LOGINFO, "# Performed Voronoi Meshing!");
				emit log(LOGINFO, tr("# Vertices: %1").arg(cMesh->n_vertices()));
				emit log(LOGINFO, tr("# Edges: %1").arg(cMesh->n_edges()));
				emit log(LOGINFO, tr("# Faces: %1").arg(cMesh->n_faces()));
				emit log(LOGINFO, "# --------------------------- #");
			} else {
				emit log(LOGERR, "Voronoi meshing failed");
				QErrorMessage error;
				error.showMessage("The voronoi meshing algorithm failed.");
				error.exec();
			}
		}
	}
}

void BezierTriangleAlgorithmsPlugin::callPartitionStep()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
		DATA_BEZIER_TRIANGLE_MESH
	);

	if (o_it != PluginFunctions::objectsEnd()) {

		BTMeshObject *meshObj = dynamic_cast<BTMeshObject*>(*o_it);

		bool ok = true;
		int seedCount;

		if (m_vinit.find(meshObj->id()) == m_vinit.end()) {

			if (applyTargetDegree(*o_it)) {
				emit updatedObject(meshObj->id(), UPDATE_ALL);
			}

			int minValue = meshObj->mesh()->n_faces()*0.01;
			seedCount = QInputDialog::getInt(
				m_tool,
				"Voronoi Meshing",
				"Please enter the minimum number of seeds: ",
				// value, min value
				minValue, 0,
				// max value, steps
				meshObj->mesh()->n_faces(), 1, &ok
			);
		}

		if (ok) {
			int ctrl_id;
			BTMeshObject *ctrlMeshObj;

			if (m_vinit.find(meshObj->id()) == m_vinit.end()) {

				emit addEmptyObject(DATA_BEZIER_TRIANGLE_MESH, ctrl_id);
				PluginFunctions::getObject(ctrl_id, ctrl_obj);
				ctrlMeshObj = PluginFunctions::btMeshObject(ctrl_obj);
				ctrlMeshObj->setName("control mesh");
				ctrlMeshObj->hide();
				ctrlMeshObj->target(false);

				betri::voronoiInit(
					*o_it,
					ctrl_obj,
					seedCount,
					m_vFlags[0]->isChecked(),
					m_vFlags[1]->isChecked(),
					m_vFlags[2]->isChecked(),
					m_vFlags[3]->isChecked(),
					m_vparam->currentIndex(),
					m_numSamples[0]->value(),
					m_vfit->currentIndex()
				);

				m_vinit.insert(meshObj->id());

				if (m_useTimer) {
					m_timer.filename(
						meshObj->path().toStdString() +
						"/voronoi-times.txt"
					);
					auto info = betri::voronoiInfo(meshObj, ctrlMeshObj);
					m_timer.start(
						info.name + " " +
						info.vertices + " " +
						info.edges + " " +
						info.faces + " " +
						info.partition
					);
				}
			} else {
				auto remesher = betri::getVoronoiObject(meshObj, ctrlMeshObj);
				remesher->useColors(m_vFlags[0]->isChecked());

				if (m_useTimer) {
					m_timer.lapStart();
				}
			}


			bool done;
			bool success = betri::voronoiPartition(*o_it, ctrl_obj, true, done);

			if (m_useTimer) {
				auto info = betri::voronoiInfo(meshObj, ctrlMeshObj);
				m_timer.lapEnd(
					"\n\tPARTITION STEP: vertices: " + info.vertices +
					"\n\tedges: " + info.edges +
					"\n\tfaces: " + info.faces
				);
			}

			if (success) {
				emit updatedObject(meshObj->id(), UPDATE_ALL);

				if (done) {
					emit log(LOGINFO, "Performed partitioning step (DONE)!");
				} else {
					emit log(LOGINFO, "Performed partitioning step!");
				}

			} else {
				emit log(LOGERR, "Voronoi partition failed");
				QErrorMessage error;
				error.showMessage("The partitioning step failed.");
				error.exec();
			}
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

		if (m_useTimer) {
			m_timer.lapStart();
		}

		bool done;
		bool success = betri::voronoiDual(*o_it, ctrl_obj, true, done);

		if (m_useTimer) {
			auto info = betri::voronoiInfo(meshObj, ctrlMeshObj);
			m_timer.lapEnd(
				"\n\tDUALIZE STEP vertices: " + info.vertices +
				"\n\tedges: " + info.edges +
				"\n\tfaces: " + info.faces
			);
		}

		emit log(LOGINFO, "Performed Dualizing Step!");

		// only show control mesh obj if its complete
		if (done) {

			if (success) {
				emit log(LOGINFO, "Finished Dualizing!");

				emit updatedObject(meshObj->id(), UPDATE_ALL);
				emit updatedObject(ctrlMeshObj->id(), UPDATE_ALL);

				// TODO: does not work for some reason
				ctrlMeshObj->mesh()->setRenderable();
				ctrlMeshObj->show();

			} else {
				emit log(LOGERR, "Dualizing failed!");
				QErrorMessage error;
				error.showMessage("The dualizing step failed.");
				error.exec();
			}
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

		if (m_useTimer) {
			m_timer.lapStart();
		}

		bool done;
		bool success = betri::voronoiDual(*o_it, ctrl_obj, false, done);

		if (m_useTimer) {
			auto info = betri::voronoiInfo(meshObj, ctrlMeshObj);
			m_timer.lapEnd(
				"\n\tDUALIZE vertices: " + info.vertices +
				"\n\tedges: " + info.edges +
				"\n\tfaces: " + info.faces
			);
		}

		if (success && done) {

			emit log(LOGINFO, "Performed Dualizing!");

			emit updatedObject(meshObj->id(), UPDATE_ALL);
			emit updatedObject(ctrlMeshObj->id(), UPDATE_ALL);

			// TODO: does not work for some reason
			ctrlMeshObj->mesh()->setRenderable();
			ctrlMeshObj->show();

		} else if (!success) {
			emit log(LOGERR, "Dualizing failed!");
			QErrorMessage error;
			error.showMessage("The dualizing failed.");
			error.exec();
		}
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

		// update values in case they were changed between steps
		auto remesher = betri::getVoronoiObject(meshObj, ctrlMeshObj);
		remesher->interpolate(m_vFlags[1]->isChecked());
		remesher->overwrite(m_vFlags[2]->isChecked());
		remesher->weights(m_vparam->currentIndex());
		remesher->fittingSamples(m_numSamples[0]->value());
		remesher->fittingSolver((betri::Fitting::Solver)m_vfit->currentIndex());

		if (m_useTimer) {
			m_timer.lapStart();
		}

		bool success = betri::voronoiFitting(*o_it, ctrl_obj);

		if (m_useTimer) {
			auto info = betri::voronoiInfo(meshObj, ctrlMeshObj);
			m_timer.end(
				"\n\tFITTING vertices: " + info.vertices +
				"\n\tedges: " + info.edges +
				"\n\tfaces: " + info.faces
			);
		}

		m_vinit.erase(meshObj->id());

		if (success) {

			BezierTMesh *cMesh = ctrlMeshObj->mesh();

			if (m_vFlags[2]->isChecked()) {
				cMesh = meshObj->mesh();
				// remove tmp object
				emit deleteObject(ctrlMeshObj->id());
				ctrl_obj = nullptr;
			} else {
				meshObj->hide();

				cMesh->setRenderable();
				ctrlMeshObj->show();
			}

			emit updatedObject(meshObj->id(), UPDATE_ALL);

			emit log(LOGINFO, "# --------------------------- #");
			emit log(LOGINFO, "# Performed Fitting!");
			emit log(LOGINFO, tr("# Vertices: %1").arg(cMesh->n_vertices()));
			emit log(LOGINFO, tr("# Edges: %1").arg(cMesh->n_edges()));
			emit log(LOGINFO, tr("# Faces: %1").arg(cMesh->n_faces()));
			emit log(LOGINFO, "# --------------------------- #");
		} else {
			emit log(LOGERR, "Fitting failed!");
			QErrorMessage error;
			error.showMessage("The fitting failed.");
			error.exec();
		}
	}

	for (auto button : m_voronoiSteps) {
		button->setDisabled(false);
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
		BTMeshObject *ctrlMeshObj;
		BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);

		bool ok = true;
		int seedCount;

		if (applyTargetDegree(*o_it)) {
			emit updatedObject(meshObj->id(), UPDATE_ALL);
		}

		int minValue = meshObj->mesh()->n_faces()*0.01;
		seedCount = QInputDialog::getInt(
			m_tool,
			"Voronoi Meshing",
			"Please enter the minimum number of seeds: ",
			// value, min value
			minValue, 0,
			// max value, steps
			meshObj->mesh()->n_faces(), 1, &ok
		);

		if (ok) {

			if (m_vinit.find(meshObj->id()) == m_vinit.end()) {

				emit addEmptyObject(DATA_BEZIER_TRIANGLE_MESH, ctrl_id);
				PluginFunctions::getObject(ctrl_id, ctrl_obj);
				BTMeshObject *ctrlMeshObj = PluginFunctions::btMeshObject(ctrl_obj);

				ctrlMeshObj->setName(meshObj->name() + "_ctrl");
				ctrlMeshObj->hide();
				ctrlMeshObj->target(false);

				betri::voronoiInit(
					*o_it,
					ctrl_obj,
					seedCount,
					m_vFlags[0]->isChecked(),
					m_vFlags[1]->isChecked(),
					m_vFlags[2]->isChecked(),
					m_vFlags[3]->isChecked(),
					m_vparam->currentIndex(),
					m_numSamples[0]->value(),
					m_vfit->currentIndex()
				);

				m_vinit.insert(meshObj->id());
			}

			if (m_useTimer) {
				m_timer.filename(
					meshObj->path().toStdString() +
					"/voronoi-times.txt"
				);
				auto info = betri::voronoiInfo(meshObj, ctrlMeshObj);
				m_timer.start(
					info.name + " " +
					info.vertices + " " +
					info.edges + " " +
					info.faces + " " +
					info.partition
				);
			}

			bool done;
			bool success = betri::voronoiPartition(*o_it, ctrl_obj, false, done);

			if (m_useTimer) {
				auto info = betri::voronoiInfo(meshObj, ctrlMeshObj);
				m_timer.lapEnd(
					"\n\tPARTITION vertices: " + info.vertices +
					"\n\tedges: " + info.edges +
					"\n\tfaces: " + info.faces
				);
			}

			if (success) {

				emit log(LOGINFO, "Performed Voronoi Partition!");

				//emit updatedObject(meshObj->id(), UPDATE_COLOR);
				emit updatedObject(meshObj->id(), UPDATE_ALL);
			} else {
				emit log(LOGERR, "Voronoi partition failed");
				QErrorMessage error;
				error.showMessage("The partition failed.");
				error.exec();
			}
		}
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

		if (applyTargetDegree(*o_it)) {
			emit updatedObject(meshObj->id(), UPDATE_ALL);
		}

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
			betri::decimationInit(
				meshObj,
				target,
				m_numSamples[1]->value(),
				m_dfit->currentIndex(),
				m_dFlags[0]->isChecked()
			);

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

		if (m_useTimer) {
			m_timer.filename(
				meshObj->path().toStdString() +
				"/decimation-times.txt"
			);
			auto info = betri::decimationInfo(meshObj);
			m_timer.start(
				info.name + " " +
				info.vertices + " " +
				info.edges + " " +
				info.faces + " " +
				info.target
			);
		}

		betri::decimation(meshObj, false, m_dFlags[1]->isChecked());

		if (m_useTimer) {
			auto info = betri::decimationInfo(meshObj);
			m_timer.end(
				"\n\tvertices: " + info.vertices +
				"\n\tedges: " + info.edges +
				"\n\tfaces: " + info.faces
			);
		}

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
			m_dFlags[1]->isChecked()
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

