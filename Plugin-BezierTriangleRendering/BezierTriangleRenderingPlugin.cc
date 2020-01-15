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
#include <qpushbutton.h>

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>
#include <OpenFlipper/common/GlobalOptions.hh>
#include <OpenFlipper/common/RendererInfo.hh>

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
	modeComboBox->addItem(tr("Ray casting"));

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
	// set default to AABB
	boundVComboBox->setCurrentIndex(1);
	PluginFunctions::betriOption(
		betri::BezierOption::BOUNDING_VOLUME,
		betri::boundingVolumeType::AABB
	);

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
	// Performance group
	///////////////////////////////////////////////////////////////////////////
	QGroupBox *perfGroup = new QGroupBox(tr("Performace Tester"));

	QLabel *filePathLabel = new QLabel(tr("outFile path:"));
	filePathComboBox = new QComboBox;
	filePathComboBox->addItem(tr("C:/Users/DoktorManto/Uni/Masterarbeit/masterthesis/rene/data/"));
	filePathComboBox->addItem(tr("D:/03_Uni/Masterarbeit/masterthesis/rene/data/"));
	filePathComboBox->addItem(tr("Custom"));
	filePathLineEdit = new QLineEdit;
	filePathLineEdit->setPlaceholderText("Placeholder Text");
	filePathLineEdit->hide();

	QLabel *outFileLabel = new QLabel(tr("Outfile name:"));
	outFileLineEdit = new QLineEdit;
	outFileLineEdit->setPlaceholderText("tmpName");

	QLabel *fileTypeLabel = new QLabel(tr("outFile type:"));
	fileTypeComboBox = new QComboBox;
	fileTypeComboBox->addItem(tr(".dat"));

	QLabel *presetsLabel = new QLabel(tr("Presets:"));
	QPushButton *testCGRButton = new QPushButton(tr("Test CGR"));
	QPushButton *testBVolsButton = new QPushButton(tr("Test Bounding Vols"));
	QPushButton *testAllButton = new QPushButton(tr("All"));

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

	QLabel *testTypeLabel = new QLabel(tr("Test type:"));
	testTypeComboBox = new QComboBox;
	testTypeComboBox->addItem(tr("Time Query"));
	testTypeComboBox->addItem(tr("Occlusion Query"));

	QLabel *againstTypeLabel = new QLabel(tr("Against type:"));
	againstTypeComboBox = new QComboBox;
	againstTypeComboBox->addItem(tr("Frame"));
	againstTypeComboBox->addItem(tr("Triangle Count"));
	againstTypeComboBox->addItem(tr("Degree"));
	againstTypeComboBox->addItem(tr("Genus"));

	QLabel *framecountLabel = new QLabel(tr("Framecount:"));
	QSpinBox *framecountSpinBox = new QSpinBox;
	framecountSpinBox->setRange(1, 10000);
	framecountSpinBox->setValue(100);

	QCheckBox *appendBox = new QCheckBox("Append");
	perfCheckboxes.push_back(appendBox);

	QCheckBox *avgBox = new QCheckBox("Average");
	perfCheckboxes.push_back(avgBox);

	QPushButton *startTestButton = new QPushButton(tr("Dew it"));

	//////////////
	// Connects //
	//////////////
	// hide/show the appropriate widgets
	connect(filePathComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
		this, [=](int index) {
		switch (index) {
			case 0: case 1:
				if (!filePathLineEdit->isHidden()) {
					filePathLineEdit->hide();
				}
				break;
			default:
				filePathLineEdit->show();
		}
	}
	);

	// Setup presets
	connect(testCGRButton, QOverload<>::of(&QPushButton::pressed),
		this, [&]() {
		(perfCheckboxes[0])->setChecked(true); // CPU
		(perfCheckboxes[1])->setChecked(true); // GPU
		(perfCheckboxes[2])->setChecked(false); // Tetrahedron
		(perfCheckboxes[3])->setChecked(false); // AABB
		(perfCheckboxes[4])->setChecked(true); // Prism
		(perfCheckboxes[5])->setChecked(false); // Chull
		(perfCheckboxes[6])->setChecked(false); // Billboard
	}
	);

	connect(testBVolsButton, QOverload<>::of(&QPushButton::pressed),
		this, [&]() {
		(perfCheckboxes[0])->setChecked(false); // CPU
		(perfCheckboxes[1])->setChecked(false); // GPU
		(perfCheckboxes[2])->setChecked(false); // Tetrahedron
		(perfCheckboxes[3])->setChecked(true); // AABB
		(perfCheckboxes[4])->setChecked(true); // Prism
		(perfCheckboxes[5])->setChecked(true); // Chull
		(perfCheckboxes[6])->setChecked(true); // Billboard
	}
	);

	connect(testAllButton, QOverload<>::of(&QPushButton::pressed),
		this, [&]() {
		(perfCheckboxes[0])->setChecked(true); // CPU
		(perfCheckboxes[1])->setChecked(true); // GPU
		(perfCheckboxes[2])->setChecked(true); // Tetrahedron
		(perfCheckboxes[3])->setChecked(true); // AABB
		(perfCheckboxes[4])->setChecked(true); // Prism
		(perfCheckboxes[5])->setChecked(true); // Chull
		(perfCheckboxes[6])->setChecked(true); // Billboard
	}
	);

	///////////////////////////////////////
	// Submit information to benchmarker //
	///////////////////////////////////////
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
			std::string tmp = filePathComboBox->currentText().toStdString();
			if (tmp.compare("Custom") == 0)
				tmp = filePathLineEdit->text().toStdString();

			PluginFunctions::ObjectIterator o_it(
				PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH
			);
			if (o_it != PluginFunctions::objectsEnd()) {
				BTMeshObject *meshObj = PluginFunctions::btMeshObject(*o_it);
				BezierTMesh *mesh = meshObj->mesh();
				std::string meshName = meshObj->name().toStdString();
				meshName[0] = toupper(meshName[0]);

				ACG::Benchmarker::instance()->meshInfo(
					meshName.substr(0, meshName.find("-", 0)),
					mesh->n_faces(), mesh->degree()
				);
			}

			// TODO put all in one function
			ACG::Benchmarker::instance()->filePath(tmp);
			ACG::Benchmarker::instance()->fileName(
				outFileLineEdit->text().toStdString()
			);
			ACG::Benchmarker::instance()->fileType(
				fileTypeComboBox->currentText().toStdString()
			);

			ACG::Benchmarker::instance()->testType(
				(ACG::Benchmarker::TEST_TYPE)testTypeComboBox->currentIndex()
			);
			ACG::Benchmarker::instance()->againstType(
				(ACG::Benchmarker::AGAINST_TYPE)againstTypeComboBox->currentIndex()
			);
			ACG::Benchmarker::instance()->append(
				(perfCheckboxes[7])->isChecked()
			);
			ACG::Benchmarker::instance()->average(
				(perfCheckboxes[8])->isChecked()
			);

			ACG::Benchmarker::instance()->active(true);
			ACG::Benchmarker::instance()->renderMode(result);
		}
	);

	///////////////////////////
	// Add widgets to layout //
	///////////////////////////

	QGridLayout *perfLayout = new QGridLayout;
	perfLayout->addWidget(filePathLabel, 0, 0);
	perfLayout->addWidget(filePathComboBox, 1, 0, 1, 3);
	perfLayout->addWidget(filePathLineEdit, 2, 0, 1, 3);
	perfLayout->addWidget(outFileLabel, 3, 0);
	perfLayout->addWidget(outFileLineEdit, 3, 1);
	perfLayout->addWidget(appendBox, 3, 2);
	perfLayout->addWidget(fileTypeLabel, 4, 0);
	perfLayout->addWidget(fileTypeComboBox, 4, 1);

	perfLayout->addWidget(presetsLabel, 5, 0);
	perfLayout->addWidget(testCGRButton, 6, 0);
	perfLayout->addWidget(testBVolsButton, 6, 1);
	perfLayout->addWidget(testAllButton, 6, 2);

	perfLayout->addWidget(cpuBox, 7, 0);
	perfLayout->addWidget(gpuBox, 7, 1);
	perfLayout->addWidget(rTetraBox, 8, 0);
	perfLayout->addWidget(rBoxBox, 8, 1);
	perfLayout->addWidget(rPrismBox, 8, 2);
	perfLayout->addWidget(rHullBox, 9, 0);
	//perfLayout->addWidget(rMeshBox, 9, 1);
	perfLayout->addWidget(rBillBox, 9, 2);

	perfLayout->addWidget(testTypeLabel, 10, 0);
	perfLayout->addWidget(testTypeComboBox, 10, 1);
	perfLayout->addWidget(againstTypeLabel, 11, 0);
	perfLayout->addWidget(againstTypeComboBox, 11, 1);

	perfLayout->addWidget(framecountLabel, 12, 0);
	perfLayout->addWidget(framecountSpinBox, 12, 1);
	perfLayout->addWidget(avgBox, 12, 2);

	perfLayout->addWidget(startTestButton, 13, 0);
	perfGroup->setLayout(perfLayout);

	////////////////////////
	// Add groups to grid //
	////////////////////////

	QGridLayout *grid = new QGridLayout();
	grid->addWidget(modeGroup, 1, 0);
	grid->addWidget(tessGroup, 2, 0);
	grid->addWidget(raytracingGroup, 3, 0);
	grid->addWidget(visGroup, 4, 0);
	grid->addWidget(perfGroup, 5, 0);
	m_tool->setLayout(grid);

    emit addToolbox(tr("Bezier Triangle Rendering"), m_tool, toolIcon);
}

void BezierTriangleRenderingPlugin::setTessType(int value)
{
	PluginFunctions::betriOption(BezierOption::TESSELLATION_TYPE, value);
	// TODO does franziska approves this?
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH
	);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleRenderingPlugin::setTessAmount(int value)
{
	PluginFunctions::betriOption(BezierOption::TESSELLATION_AMOUNT, value);
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH
	);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleRenderingPlugin::setTessMode(int value)
{
	PluginFunctions::betriOption(BezierOption::TESSELLATION_ADAPTIVE, value);
	// TODO does franziska approves this?
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH
	);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleRenderingPlugin::setBoundVType(int value)
{
	PluginFunctions::betriOption(BezierOption::BOUNDING_VOLUME, value);
	// TODO does franziska approves this?
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH
	);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleRenderingPlugin::setVisulisationType(int value)
{
	PluginFunctions::betriOption(BezierOption::VISUALISATION_MODE, value);
	// TODO does franziska approves this?
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH
	);
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
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH
	);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleRenderingPlugin::setDError(int value)
{
	PluginFunctions::betriOption(BezierOption::D_ERROR, value);
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH
	);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}

void BezierTriangleRenderingPlugin::setNewtonIt(int value)
{
	PluginFunctions::betriOption(BezierOption::NEWTON_IT_COUNT, value);
	PluginFunctions::ObjectIterator o_it(
		PluginFunctions::ALL_OBJECTS, DATA_BEZIER_TRIANGLE_MESH
	);
	for (; o_it != PluginFunctions::objectsEnd(); ++o_it) {
		emit updatedObject(o_it->id(), UPDATE_GEOMETRY);
	}
}
