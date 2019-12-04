#include "BezierTriangleUtilsPlugin.hh"

#include <qpushbutton.h>
#include <qgridlayout.h>
// TODO new
#include <qgroupbox.h>
#include <qcombobox.h>
#include <qlabel.h>
#include <qspinbox.h>
#include <qdoublespinbox>
#include <qcheckbox.h>
#include <qinputdialog.h>

//#include <OpenFlipper/common/UpdateType.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <OpenFlipper/common/GlobalOptions.hh>
#include <OpenFlipper/common/RendererInfo.hh>
#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>

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

	QGridLayout *voronoiLayout = new QGridLayout;
	voronoiLayout->addWidget(voronoiButton, 0, 0, 1, 3);
	voronoiLayout->addWidget(patitionButton, 1, 0, 2, 1);
	voronoiLayout->addWidget(dualStepButton, 1, 1, 1, 1);
	voronoiLayout->addWidget(dualButton, 2, 1, 1, 1);
	voronoiLayout->addWidget(fittingButton, 1, 2, 2, 1);
	voronoiLayout->addWidget(smoothButton, 3, 0, 1, 3);
	voronoiGroup->setLayout(voronoiLayout);

	QPushButton *testButton = new QPushButton(tr("Start Test"));
	connect(testButton, SIGNAL(clicked()), this, SLOT(testAlgorithm()));

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
	// Visualisation mode group
	///////////////////////////////////////////////////////////////////////////

	QGroupBox *modeGroup = new QGroupBox(tr("Mode"));

	// Create Drop down menu
	QLabel *modeLabel = new QLabel(tr("Mode:"));
	QComboBox *modeComboBox = new QComboBox;
	modeComboBox->addItem(tr("NONE"));
	modeComboBox->addItem(tr("CPU"));
	modeComboBox->addItem(tr("GPU"));
	modeComboBox->addItem(tr("Raytracing"));

	// set the connection to the setTessType
	// TODO warum ist das hier anders als bei der tessspinbox
	connect(modeComboBox, QOverload<int>::of(&QComboBox::activated),
		this, &BezierTriangleUtilsPlugin::setTessType);

	QGridLayout *modeLayout = new QGridLayout;
	modeLayout->addWidget(modeLabel, 0, 0);
	modeLayout->addWidget(modeComboBox, 0, 1);
	modeGroup->setLayout(modeLayout);

	///////////////////////////////////////////////////////////////////////////
	// Tesselation Settings group
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *tessGroup = new QGroupBox(tr("Tessellation"));

	// Create the gui for tessamount selection
	QLabel *tessALabel = new QLabel(tr("TesselationAmount:"));
	QSpinBox *tessSpinBox = new QSpinBox;
	QSlider *tessSlider = new QSlider(Qt::Horizontal);
	tessSpinBox->setRange(0, 10);
	tessSlider->setRange(0, 10);

	connect(tessSpinBox, SIGNAL(valueChanged(int)), tessSlider, SLOT(setValue(int)));
	connect(tessSlider, SIGNAL(valueChanged(int)), tessSpinBox, SLOT(setValue(int)));
	connect(tessSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setTessAmount(int)));

	tessSpinBox->setValue(0);

	// Create Drop down menu
	QLabel *tessMLabel = new QLabel(tr("Tesselation Mode:"));
	QComboBox *tessmodeComboBox = new QComboBox;
	tessmodeComboBox->addItem(tr("Constant"));
	tessmodeComboBox->addItem(tr("Dist-Adaptive"));
	tessmodeComboBox->addItem(tr("Flatness-Adaptive"));

	connect(tessmodeComboBox, QOverload<int>::of(&QComboBox::activated),
		this, &BezierTriangleUtilsPlugin::setTessMode);

	// hide/show the appropriate widgets
	connect(modeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, [=](int index) {
			switch (index) {
				case betri::TESSELLATION_TYPE::NONE:
				case betri::TESSELLATION_TYPE::RAYTRACING:
					if (tessGroup->isVisible()) {
						tessGroup->hide();
					}
					break;
				case betri::TESSELLATION_TYPE::CPU:
					if (tessGroup->isHidden()) {
						tessGroup->show();
					}
					tessMLabel->hide();
					tessmodeComboBox->hide();
					break;
				case betri::TESSELLATION_TYPE::GPU:
					if (tessGroup->isHidden()) {
						tessGroup->show();
					}
					tessMLabel->show();
					tessmodeComboBox->show();
					break;
				default:
					tessGroup->hide();
			}
		}
	);

	QGridLayout *tessLayout = new QGridLayout;
	tessLayout->addWidget(tessALabel, 0, 0);
	tessLayout->addWidget(tessSpinBox, 0, 1);
	tessLayout->addWidget(tessSlider, 1, 0);
	tessLayout->addWidget(tessMLabel, 2, 0);
	tessLayout->addWidget(tessmodeComboBox, 3, 0);
	tessGroup->setLayout(tessLayout);
	tessGroup->hide();

	///////////////////////////////////////////////////////////////////////////
	// Raytracing Settings group
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *raytracingGroup = new QGroupBox(tr("Raytracing"));

	QLabel *boundVLabel = new QLabel(tr("Use BoundingVolume:"));
	QComboBox *boundVComboBox = new QComboBox;
	//boundVComboBox->addItem(tr("NONE"));
	boundVComboBox->addItem(tr("AABB")); // TODO the value should be betri::beziermathutil.hh::PRISM ...
	boundVComboBox->addItem(tr("PRISM"));
	boundVComboBox->addItem(tr("BOUNDING Tetraeder"));
	boundVComboBox->addItem(tr("CONVEX HULL"));
	boundVComboBox->addItem(tr("BOUNDING MESH"));
	boundVComboBox->addItem(tr("BOUNDING BILLBOARD"));

	QLabel *berrorLabel = new QLabel(tr("bary-Error:"));
	QSpinBox *berrorSpinBox = new QSpinBox;
	berrorSpinBox->setRange(0, 10000);

	QLabel *derrorLabel = new QLabel(tr("dot-error:"));
	QSpinBox *derrorSpinBox = new QSpinBox;
	derrorSpinBox->setRange(0, 10000);

	QLabel *niterationLabel = new QLabel(tr("Newton Iteration Count:"));
	QSpinBox *niterationSpinBox = new QSpinBox;
	niterationSpinBox->setRange(0, 20);

	connect(berrorSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setBError(int)));
	connect(derrorSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setDError(int)));
	connect(niterationSpinBox, SIGNAL(valueChanged(int)), this, SLOT(setNewtonIt(int)));

	berrorSpinBox->setValue(10000);
	derrorSpinBox->setValue(10000);
	niterationSpinBox->setValue(6);

	// hide/show the appropriate widgets
	connect(modeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, [=](int index) {
		switch (index) {
			case betri::TESSELLATION_TYPE::RAYTRACING:
				if (raytracingGroup->isHidden()) {
					raytracingGroup->show();
				}
				break;
			default:
				raytracingGroup->hide();
		}
	}
	);

	connect(boundVComboBox, QOverload<int>::of(&QComboBox::activated),
		this, &BezierTriangleUtilsPlugin::setBoundVType);

	QGridLayout *rayLayout = new QGridLayout;
	rayLayout->addWidget(boundVLabel, 0, 0);
	rayLayout->addWidget(boundVComboBox, 0, 1);
	rayLayout->addWidget(berrorLabel, 1, 0);
	rayLayout->addWidget(berrorSpinBox, 1, 1);
	rayLayout->addWidget(derrorLabel, 2, 0);
	rayLayout->addWidget(derrorSpinBox, 2, 1);
	rayLayout->addWidget(niterationLabel, 3, 0);
	rayLayout->addWidget(niterationSpinBox, 3, 1);
	raytracingGroup->setLayout(rayLayout);
	raytracingGroup->hide();

	///////////////////////////////////////////////////////////////////////////
	// Visualisation group
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *visGroup = new QGroupBox(tr("Visualisation"));

	QCheckBox *cullBox = new QCheckBox("Backfaceculling");
	QCheckBox *boundVBox = new QCheckBox("Show BoundingVolume");
	QLabel *visLabel = new QLabel(tr("Facecoloring:"));
	QComboBox *visComboBox = new QComboBox;
	visComboBox->addItem(tr("Phong-Color"));
	visComboBox->addItem(tr("Color"));
	visComboBox->addItem(tr("Normal"));
	visComboBox->addItem(tr("Depth"));
	visComboBox->addItem(tr("UV"));
	visComboBox->addItem(tr("Curvature"));

	// hide/show the appropriate widgets
	connect(modeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, [=](int index) {
		switch (index) {
			case betri::TESSELLATION_TYPE::RAYTRACING:
				if (boundVBox->isHidden()) {
					boundVBox->show();
				}
				if (visComboBox->isHidden()) {
					visComboBox->show();
				}
				if (visLabel->isHidden()) {
					visLabel->show();
				}
				break;
			default:
				boundVBox->hide();
				visLabel->hide();
				visComboBox->hide();
		}
	}
	);

	connect(cullBox, QOverload<bool>::of(&QCheckBox::clicked),
		this, &BezierTriangleUtilsPlugin::setCulling);

	connect(boundVBox, QOverload<bool>::of(&QCheckBox::clicked),
		this, &BezierTriangleUtilsPlugin::setBoundVShow);

	connect(visComboBox, QOverload<int>::of(&QComboBox::activated),
		this, &BezierTriangleUtilsPlugin::setVisulisationType);

	QGridLayout *visLayout = new QGridLayout;
	visLayout->addWidget(cullBox, 0, 0);
	visLayout->addWidget(boundVBox, 1, 0);
	visLayout->addWidget(visLabel, 2, 0);
	visLayout->addWidget(visComboBox, 3, 1);
	visGroup->setLayout(visLayout);

	boundVBox->hide();
	visLabel->hide();
	visComboBox->hide();

	///////////////////////////////////////////////////////////////////////////
	// Mesh-Attribute group
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *attrGroup = new QGroupBox(tr("Mesh Attributes"));

	QPushButton *addTexCoordsButton = new QPushButton(tr("Re-Add TexCoords"));

	connect(addTexCoordsButton, QOverload<>::of(&QPushButton::pressed),
		this, [&]() {

		PluginFunctions::ObjectIterator o_it(
			PluginFunctions::TARGET_OBJECTS,
			DATA_BEZIER_TRIANGLE_MESH
		);

		if (o_it != PluginFunctions::objectsEnd()) {

			BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);
			betri::randomMeshUV(*(meshObj->mesh()));

			emit updatedObject(meshObj->id(), UPDATE_ALL);
		}
	});
	//connect(addTexCoordsButton, QOverload<>::of(&QPushButton::pressed),
	//	this, &BezierTriangleUtilsPlugin::setVisulisationType);

	QGridLayout *attrLayout = new QGridLayout;
	attrLayout->addWidget(addTexCoordsButton, 0, 0);
	attrGroup->setLayout(attrLayout);

	///////////////////////////////////////////////////////////////////////////
	// Performance group
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *perfGroup = new QGroupBox(tr("Performace Tester"));

	QPushButton *startTestButton = new QPushButton(tr("Start"));
	QPushButton *endTestButton = new QPushButton(tr("End"));

	int result;

	// https://www.openflipper.org/media/Documentation/OpenFlipper-1.2/classBaseInterface.html#ace0d6b943ce94f48c40e8c0e17a8413e
	connect(startTestButton, QOverload<>::of(&QPushButton::pressed),
		this, [&]() {

		std::cerr << "\nREDRAW DISABLED " << OpenFlipper::Options::redrawDisabled() << std::endl;
		std::cerr << "\nREDRAW loadingSettings " << OpenFlipper::Options::sceneGraphUpdatesBlocked() << std::endl;
		std::cerr << "\nREDRAW loadingSettings " << OpenFlipper::Options::examinerWidgets() << std::endl;


		std::clock_t start = std::clock();
		int frames = 6000;

		std::cerr << frames << std::endl;

		for (int i = 0; i < frames; i++) {
			// https://www.openflipper.org/media/Documentation/OpenFlipper-1.2/baseInterfacePage.html
			// http://openflipper.org/Documentation/latest/a14811.html#baseInterfaceSceneUpdateNotification
			// http://www.openflipper.org/media/Documentation/OpenFlipper-4.0/a00293_source.html
			//emit BaseInterface::updateView();
			emit updateView();
			//BaseInterface::updatedObject(-1);
			//BaseInterface::nodeVisibilityChanged(-1);

			// qtbaseviewer.cc frame_time_
			// http://www.openflipper.org/media/Documentation/OpenFlipper-1.1/classACG_1_1QtWidgets_1_1QtBaseViewer.html
			//auto bla = PluginFunctions::getRootNode();
			//bla->draw(ACG::GLState(), ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED);
			//bla->updateGL();
		}

		double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

		std::cerr << "duration: " << duration << " " << (frames/duration) << '\n';
		result = int(frames / duration);
	}
	);

	QGridLayout *perfLayout = new QGridLayout;
	perfLayout->addWidget(startTestButton, 0, 0);
	perfLayout->addWidget(endTestButton, 1, 0);
	perfGroup->setLayout(perfLayout);


	///////////////////////////////////////////////////////////////////////////
	// Add all Elements
	///////////////////////////////////////////////////////////////////////////

	QPushButton *applyTess = new QPushButton("Apply Tessellation");
	connect(applyTess, SIGNAL(clicked()), this, SLOT(applyTessellation()));

	QGridLayout *grid = new QGridLayout();
	grid->addWidget(voronoiGroup, 0, 0);
	grid->addWidget(deciGroup, 1, 0);
	grid->addWidget(testButton, 2, 0);
	grid->addWidget(applyTess, 2, 1);

	grid->addWidget(modeGroup, 3, 0);
	grid->addWidget(tessGroup, 4, 0);
	grid->addWidget(raytracingGroup, 5, 0);
	grid->addWidget(visGroup, 6, 0);
	grid->addWidget(attrGroup, 7, 0);
	grid->addWidget(perfGroup, 8, 0);
	m_tool->setLayout(grid);

    emit addToolbox(tr("Bezier Triangle Utils"), m_tool, toolIcon);
}

void BezierTriangleUtilsPlugin::tessellateMesh()
{
	emit log(LOGINFO, "Tessellated Bezier Triangles!");
}

void BezierTriangleUtilsPlugin::setTessType(int value)
{
	PluginFunctions::betriOption(BezierOption::TESSELLATION_TYPE, value);
	// TODO does franziska approves this?
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}

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

void BezierTriangleUtilsPlugin::setTessAmount(int value)
{
	PluginFunctions::betriOption(BezierOption::TESSELLATION_AMOUNT, value);
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
	emit log(LOGINFO, tr("set tessellation amount to %1").arg(value));
}

void BezierTriangleUtilsPlugin::setTessMode(int value)
{
	PluginFunctions::betriOption(BezierOption::TESSELLATION_ADAPTIVE, value);
	// TODO does franziska approves this?
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
	emit log(LOGINFO, tr("set tessellation mode to %1").arg(value));
}

void BezierTriangleUtilsPlugin::setBoundVType(int value)
{
	PluginFunctions::betriOption(BezierOption::BOUNDING_VOLUME, value);
	// TODO does franziska approves this?
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleUtilsPlugin::setVisulisationType(int value)
{
	PluginFunctions::betriOption(BezierOption::VISUALISATION_MODE, value);
	// TODO does franziska approves this?
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleUtilsPlugin::setCulling(bool value)
{
	PluginFunctions::betriOption(BezierOption::CULL_FACES, value);
}

void BezierTriangleUtilsPlugin::setBoundVShow(bool value)
{
	PluginFunctions::betriOption(BezierOption::SHOW_BOUNDING_VOLUME, value);
}

void BezierTriangleUtilsPlugin::setBError(int value)
{
	PluginFunctions::betriOption(BezierOption::B_ERROR, value);
	emit log(LOGINFO, tr("set B_ERROR to %1").arg(value));
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleUtilsPlugin::setDError(int value)
{
	PluginFunctions::betriOption(BezierOption::D_ERROR, value);
	emit log(LOGINFO, tr("set D_ERROR to %1").arg(value));
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleUtilsPlugin::setNewtonIt(int value)
{
	PluginFunctions::betriOption(BezierOption::NEWTON_IT_COUNT, value);
	emit log(LOGINFO, tr("set D_ERROR to %1").arg(value));
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
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

		BTMeshObject *meshObj = dynamic_cast<BTMeshObject*>(*o_it);

		bool ok;
		int minValue = meshObj->mesh()->n_faces()*0.01;
		int seedCount = QInputDialog::getInt(
			m_tool,
			"Voronoi Meshing",
			"Please enter the minimum number of seeds: ",
			// value, min value
			minValue, std::min(minValue,10),
			// max value, steps
			meshObj->mesh()->n_faces(), 1, &ok
		);

		if (ok) {
			betri::voronoiInit(*o_it, ctrl_obj, seedCount, true);
			betri::voronoiRemesh(*o_it, ctrl_obj);
			emit log(LOGINFO, "Performed Voronoi Remeshing!");

			emit updatedObject(meshObj->id(), UPDATE_ALL);
			emit updatedObject(ctrl_id, UPDATE_ALL);

			meshObj->hide();
			ctrlMeshObj->mesh()->setRenderable();
			ctrlMeshObj->show();
		}
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

		ctrlMeshObj->mesh()->setRenderable();

		emit updatedObject(meshObj->id(), UPDATE_ALL);
		emit updatedObject(ctrlMeshObj->id(), UPDATE_ALL);

		meshObj->hide();
		ctrlMeshObj->show();
	}
	for (auto button : m_voronoiSteps) {
		button->setDisabled(false);
	}

}

void BezierTriangleUtilsPlugin::callSmooth()
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

		bool ok;
		int minValue = meshObj->mesh()->n_faces()*0.01;
		int seedCount = QInputDialog::getInt(
			m_tool,
			"Voronoi Meshing",
			"Please enter the minimum number of seeds: ",
			// value, min value
			minValue, std::min(minValue, 10),
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

void BezierTriangleUtilsPlugin::applyTessellation()
{
	// init object iterator
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::TARGET_OBJECTS,
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

			TriMeshObject* object(0);

			emit addEmptyObject(DATA_TRIANGLE_MESH, tri_id);
			PluginFunctions::getObject(tri_id, object);

			mesh->tessellateToTrimesh(*object->mesh(), amount);

			emit updatedObject(tri_id, UPDATE_ALL);
		}
	}
}

void BezierTriangleUtilsPlugin::testAlgorithm()
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

		bool okay = true;

		int meshId = meshObj->id();
		auto data = m_target.find(meshId);

		if (data == m_target.end()) {
			m_target[meshId] = QInputDialog::getInt(
				m_tool,
				"Decimation Meshing",
				"Please enter target complexity: ",
				// value, min value
				mesh->n_vertices(), 10,
				// max value, steps
				mesh->n_vertices(), 1, &okay
			);
		}

		if (okay) {
			betri::decimation(meshObj, m_target[meshId], false);
			emit log(LOGINFO, "Performed Decimation (DONE)!");

			emit updatedObject(meshObj->id(), UPDATE_ALL);
		}
		m_target.erase(meshId);
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

		bool okay = true;

		int meshId = meshObj->id();
		auto data = m_target.find(meshId);

		if (data == m_target.end()) {
			BezierTMesh *mesh = meshObj->mesh();

			m_target[meshId] = QInputDialog::getInt(
				m_tool,
				"Decimation Meshing",
				"Please enter target complexity: ",
				// value, min value
				mesh->n_vertices(), 10,
				// max value, steps
				mesh->n_vertices(), 1, &okay
			);
		}

		if (okay) {
			const bool done = betri::decimation(meshObj, m_target[meshId], true);
			if (done) {
				emit log(LOGINFO, "Performed Decimation Step (DONE)!");
			} else {
				emit log(LOGINFO, "Performed Decimation Step!");
			}
		} else {
			m_target.erase(meshId);
		}

		emit updatedObject(meshObj->id(), UPDATE_ALL);
	}
}

