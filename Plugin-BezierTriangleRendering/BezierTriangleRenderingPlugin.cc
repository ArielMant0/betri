#include "BezierTriangleRenderingPlugin.hh"

#include <qpushbutton.h>
#include <qgridlayout.h>
// TODO new
#include <qgroupbox.h>
#include <qcombobox.h>
#include <qlabel.h>
#include <qspinbox.h>
#include <qdoublespinbox>
#include <qinputdialog.h>

#include <OpenFlipper/common/GlobalOptions.hh>
#include <OpenFlipper/common/RendererInfo.hh>

#include "OpenFlipper/BasePlugin/PluginFunctions.hh"

#include "BezierTriangleRendering.hh"

using namespace betri;

void BezierTriangleRenderingPlugin::initializePlugin()
{

	m_tool = new QWidget();
	QIcon *toolIcon = new QIcon(
		OpenFlipper::Options::iconDirStr() +
		OpenFlipper::Options::dirSeparator() +
		"btrendering.png"
	);

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
		this, &BezierTriangleRenderingPlugin::setTessType);

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
		this, &BezierTriangleRenderingPlugin::setTessMode);

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
	boundVComboBox->addItem(tr("BOUNDING Tetraeder"));
	boundVComboBox->addItem(tr("AABB")); // TODO the value should be betri::beziermathutil.hh::PRISM ...
	boundVComboBox->addItem(tr("PRISM"));
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
		this, &BezierTriangleRenderingPlugin::setBoundVType);

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
		this, &BezierTriangleRenderingPlugin::setCulling);

	connect(boundVBox, QOverload<bool>::of(&QCheckBox::clicked),
		this, &BezierTriangleRenderingPlugin::setBoundVShow);

	connect(visComboBox, QOverload<int>::of(&QComboBox::activated),
		this, &BezierTriangleRenderingPlugin::setVisulisationType);

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
	//	this, &BezierTriangleRenderingPlugin::setVisulisationType);

	QGridLayout *attrLayout = new QGridLayout;
	attrLayout->addWidget(addTexCoordsButton, 0, 0);
	attrGroup->setLayout(attrLayout);

	///////////////////////////////////////////////////////////////////////////
	// Performance group
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *perfGroup = new QGroupBox(tr("Performace Tester"));

	QPushButton *startTestButton = new QPushButton(tr("Do it"));
	QLabel *framecountLabel = new QLabel(tr("Framecount:"));
	QSpinBox *framecountSpinBox = new QSpinBox;
	framecountSpinBox->setRange(1, 1000);

	QCheckBox *cpuBox = new QCheckBox("CPU");
	perfCheckboxes.push_back(cpuBox);
	QCheckBox *gpuBox = new QCheckBox("GPU");
	perfCheckboxes.push_back(gpuBox);
	QCheckBox *rTetraBox = new QCheckBox("Ray-Tetra");
	perfCheckboxes.push_back(rTetraBox);
	QCheckBox *rBoxBox = new QCheckBox("Ray-AABB");
	perfCheckboxes.push_back(rBoxBox);
	QCheckBox *rPrismBox = new QCheckBox("Ray-Prism");
	perfCheckboxes.push_back(rPrismBox);
	QCheckBox *rHullBox = new QCheckBox("Ray-CHull");
	perfCheckboxes.push_back(rHullBox);
	QCheckBox *rBillBox = new QCheckBox("Ray-Billb");
	perfCheckboxes.push_back(rBillBox);

	// Random unrelated links yay
	// https://www.openflipper.org/media/Documentation/OpenFlipper-1.2/classBaseInterface.html#ace0d6b943ce94f48c40e8c0e17a8413e
	// https://www.openflipper.org/media/Documentation/OpenFlipper-1.2/baseInterfacePage.html
	// http://openflipper.org/Documentation/latest/a14811.html#baseInterfaceSceneUpdateNotification
	// http://www.openflipper.org/media/Documentation/OpenFlipper-4.0/a00293_source.html
	connect(startTestButton, QOverload<>::of(&QPushButton::pressed),
		this, [&]() {
			int result = 0;
			if (ACG::Benchmarker::instance()->active())
				return;

			if ((perfCheckboxes[0])->isChecked()) {
				result |= int(ACG::Benchmarker::RENDER_MODE::CPU);
			}
			if ((perfCheckboxes[1])->isChecked()) {
				result |= int(ACG::Benchmarker::RENDER_MODE::GPU);
			}
			if ((perfCheckboxes[2])->isChecked()) {
				result |= int(ACG::Benchmarker::RENDER_MODE::RAYTETRA);
			}
			if ((perfCheckboxes[3])->isChecked()) {
				result |= int(ACG::Benchmarker::RENDER_MODE::RAYAABB);
			}
			if ((perfCheckboxes[4])->isChecked()) {
				result |= int(ACG::Benchmarker::RENDER_MODE::RAYPRISM);
			}
			if ((perfCheckboxes[5])->isChecked()) {
				result |= int(ACG::Benchmarker::RENDER_MODE::RAYCHULL);
			}
			if (false) { // TODO
				result |= int(ACG::Benchmarker::RENDER_MODE::RAYMESH);
			}
			if ((perfCheckboxes[6])->isChecked()) {
				result |= int(ACG::Benchmarker::RENDER_MODE::RAYBILLB);
			}
			ACG::Benchmarker::instance()->active(true);
			ACG::Benchmarker::instance()->renderMode(result);
			if (false) {
				ACG::Benchmarker::instance()->occlQuery(false);
			}
			if (false) {
				ACG::Benchmarker::instance()->average(false);
			}
		}
	);

	QGridLayout *perfLayout = new QGridLayout;
	perfLayout->addWidget(startTestButton, 0, 0);
	perfLayout->addWidget(cpuBox, 1, 0);
	perfLayout->addWidget(gpuBox, 2, 0);
	perfLayout->addWidget(rTetraBox, 3, 0);
	perfLayout->addWidget(rBoxBox, 4, 0);
	perfLayout->addWidget(rPrismBox, 5, 0);
	perfLayout->addWidget(rHullBox, 6, 0);
	perfLayout->addWidget(rBillBox, 7, 0);
	perfGroup->setLayout(perfLayout);

	///////////////////////////////////////////////////////////////////////////
	// Add all Elements
	///////////////////////////////////////////////////////////////////////////

	QGridLayout *grid = new QGridLayout();
	grid->addWidget(modeGroup, 1, 0);
	grid->addWidget(tessGroup, 2, 0);
	grid->addWidget(raytracingGroup, 3, 0);
	grid->addWidget(visGroup, 4, 0);
	grid->addWidget(attrGroup, 5, 0);
	grid->addWidget(perfGroup, 6, 0);
	m_tool->setLayout(grid);

    emit addToolbox(tr("Bezier Triangle Rendering"), m_tool, toolIcon);
}

void BezierTriangleRenderingPlugin::setTessType(int value)
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

void BezierTriangleRenderingPlugin::setTessAmount(int value)
{
	PluginFunctions::betriOption(BezierOption::TESSELLATION_AMOUNT, value);
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
	emit log(LOGINFO, tr("set tessellation amount to %1").arg(value));
}

void BezierTriangleRenderingPlugin::setTessMode(int value)
{
	PluginFunctions::betriOption(BezierOption::TESSELLATION_ADAPTIVE, value);
	// TODO does franziska approves this?
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
	emit log(LOGINFO, tr("set tessellation mode to %1").arg(value));
}

void BezierTriangleRenderingPlugin::setBoundVType(int value)
{
	PluginFunctions::betriOption(BezierOption::BOUNDING_VOLUME, value);
	// TODO does franziska approves this?
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleRenderingPlugin::setVisulisationType(int value)
{
	PluginFunctions::betriOption(BezierOption::VISUALISATION_MODE, value);
	// TODO does franziska approves this?
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleRenderingPlugin::setCulling(bool value)
{
	PluginFunctions::betriOption(BezierOption::CULL_FACES, value);
}

void BezierTriangleRenderingPlugin::setBoundVShow(bool value)
{
	PluginFunctions::betriOption(BezierOption::SHOW_BOUNDING_VOLUME, value);
}

void BezierTriangleRenderingPlugin::setBError(int value)
{
	PluginFunctions::betriOption(BezierOption::B_ERROR, value);
	emit log(LOGINFO, tr("set B_ERROR to %1").arg(value));
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleRenderingPlugin::setDError(int value)
{
	PluginFunctions::betriOption(BezierOption::D_ERROR, value);
	emit log(LOGINFO, tr("set D_ERROR to %1").arg(value));
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleRenderingPlugin::setNewtonIt(int value)
{
	PluginFunctions::betriOption(BezierOption::NEWTON_IT_COUNT, value);
	emit log(LOGINFO, tr("set D_ERROR to %1").arg(value));
	PluginFunctions::ObjectIterator o_it(PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}
