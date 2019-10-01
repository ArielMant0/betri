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

	QPushButton *voronoiButton = new QPushButton(tr("Do it"));
	connect(voronoiButton, SIGNAL(clicked()), this, SLOT(callVoronoi()));

	QGridLayout *voronoiLayout = new QGridLayout;
	voronoiLayout->addWidget(voronoiButton, 0, 0);
	voronoiGroup->setLayout(voronoiLayout);

	///////////////////////////////////////////////////////////////////////////
	// Mode group
	// https://doc.qt.io/qt-5/qtwidgets-widgets-lineedits-example.html	
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
	// Tesselation group
	// https://doc.qt.io/qt-5/qtwidgets-widgets-lineedits-example.html
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

	tessSpinBox->setValue(1);

	QCheckBox *adaptCBox = new QCheckBox("Adaptive Tesselation");

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
				case 1: case 2:
					if (tessGroup->isHidden()) {
						tessGroup->show();
					}
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
	tessLayout->addWidget(adaptCBox, 2, 0);
	tessGroup->setLayout(tessLayout);
	tessGroup->hide();

	///////////////////////////////////////////////////////////////////////////
	// Raytracing group
	// https://doc.qt.io/qt-5/qtwidgets-widgets-lineedits-example.html
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *raytracingGroup = new QGroupBox(tr("Raytracing"));

	QLabel *boundVLabel = new QLabel(tr("Use BoundingVolume:"));
	QComboBox *boundVComboBox = new QComboBox;
	//boundVComboBox->addItem(tr("NONE"));
	boundVComboBox->addItem(tr("AABB")); // TODO the value should be betri::beziermathutil.hh::PRISM ...
	boundVComboBox->addItem(tr("PRISM"));

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
	raytracingGroup->setLayout(rayLayout);
	raytracingGroup->hide();

	///////////////////////////////////////////////////////////////////////////
	// Visualisation group
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *visGroup = new QGroupBox(tr("Visualisation"));

	QCheckBox *boundVBox = new QCheckBox("Show BoundingVolume");

	// hide/show the appropriate widgets
	connect(modeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, [=](int index) {
		switch (index) {
			case betri::TESSELLATION_TYPE::RAYTRACING:
				if (boundVBox->isHidden()) {
					boundVBox->show();
				}
				break;
			default:
				boundVBox->hide();
		}
	}
	);

	connect(boundVBox, QOverload<bool>::of(&QCheckBox::clicked),
		this, &BezierTriangleUtilsPlugin::setBoundVShow);

	QGridLayout *visLayout = new QGridLayout;
	visLayout->addWidget(boundVBox, 0, 0);
	visGroup->setLayout(visLayout);

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

		std::clock_t start = std::clock();
		int frames = 6000;

		std::cerr << frames << std::endl;

		for (int i = 0; i < frames; i++) {
			// https://www.openflipper.org/media/Documentation/OpenFlipper-1.2/baseInterfacePage.html
			// http://www.openflipper.org/media/Documentation/OpenFlipper-4.0/a00293_source.html
			BaseInterface::updateView();
			BaseInterface::updatedObject(-1);
			BaseInterface::nodeVisibilityChanged(-1);

			// qtbaseviewer.cc frame_time_
			// http://www.openflipper.org/media/Documentation/OpenFlipper-1.1/classACG_1_1QtWidgets_1_1QtBaseViewer.html
			auto bla = PluginFunctions::getRootNode();
			bla->draw(ACG::GLState(), ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED);
		}

		double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

		std::cerr << "duration: " << duration << " " << (frames/duration) << '\n';
		result = int(frames / duration);
	}
	);

	std::cerr << "result: " << result << std::endl;


	QGridLayout *perfLayout = new QGridLayout;
	perfLayout->addWidget(startTestButton, 0, 0);
	perfLayout->addWidget(endTestButton, 1, 0);
	perfGroup->setLayout(perfLayout);

	///////////////////////////////////////////////////////////////////////////
	// Add all Elements
	///////////////////////////////////////////////////////////////////////////
	QGridLayout *grid = new QGridLayout();
	grid->addWidget(voronoiGroup, 0, 0);
	grid->addWidget(modeGroup, 1, 0);
	grid->addWidget(tessGroup, 2, 0);
	grid->addWidget(raytracingGroup, 3, 0);
	grid->addWidget(visGroup, 4, 0);
	grid->addWidget(perfGroup, 5, 0);
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
	// TODO does franziska approves this?
	PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
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

void BezierTriangleUtilsPlugin::setBoundVType(int value)
{
	PluginFunctions::betriOption(BezierOption::BOUNDING_VOLUME, value);
	// TODO does franziska approves this?
	PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleUtilsPlugin::setBoundVShow(bool value)
{
	PluginFunctions::betriOption(BezierOption::SHOW_BOUNDING_VOLUME, value);
}

void BezierTriangleUtilsPlugin::callVoronoi()
{
	PluginFunctions::ObjectIterator o_it(PluginFunctions::TARGET_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {

		int ctrl_id;
		BaseObjectData *obj;
		BTMeshObject *ctrl_obj;

		emit addEmptyObject(DATA_BEZIER_TRIANGLE_MESH, ctrl_id);
		PluginFunctions::getObject(ctrl_id, obj);
		ctrl_obj = PluginFunctions::btMeshObject(obj);
		ctrl_obj->setName("control mesh");
		ctrl_obj->hide();
		ctrl_obj->target(false);

		betri::voronoiRemesh(*o_it, obj);
		emit log(LOGINFO, "Performed Voronoi Partition!");

		BTMeshObject *meshObj = dynamic_cast<BTMeshObject*>(*o_it);
		emit updatedObject(meshObj->id(), UPDATE_ALL);
		emit updatedObject(ctrl_id, UPDATE_ALL);
	}
}

