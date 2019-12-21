#pragma once

#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/ToolboxInterface.hh>
#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/BasePlugin/RenderInterface.hh>
#include <OpenFlipper/BasePlugin/LoadSaveInterface.hh>

#include <OpenFlipper/common/Types.hh>

#include <qlineedit.h> // TODO
#include <qpushbutton.h> // TODO
#include <qcheckbox.h>

class BezierTriangleAlgorithmsPlugin : public QObject,
	BaseInterface,
	ToolboxInterface,
	LoggingInterface,
	LoadSaveInterface
{
	Q_OBJECT
	Q_INTERFACES(BaseInterface)
	Q_INTERFACES(ToolboxInterface)
	Q_INTERFACES(LoggingInterface)
	Q_INTERFACES(LoadSaveInterface)

	Q_PLUGIN_METADATA(IID "org.OpenFlipper.Plugins.Plugin-BezierTriangleAlgorithms")

public:

	BezierTriangleAlgorithmsPlugin() : m_tool(0) {}

	~BezierTriangleAlgorithmsPlugin() {};

	QString name() { return QString("BezierTriangleAlgorithmsPlugin"); };

	QString description()
	{
		return QString("Construct Bezier triangle meshes from triangle meshes");
	};

private:

	QWidget *m_tool;
	std::vector<QPushButton*> m_voronoiSteps;
	std::array<QCheckBox*, 2> m_flags;
	std::array<QCheckBox*, 2> m_colors;

	BaseObjectData *ctrl_obj;

///////////////////////////////////////////////////////////////////////////////
// Signals
///////////////////////////////////////////////////////////////////////////////
signals:

	void updateView();
    void updatedObject(int _identifier, const UpdateType &type);

	void log(Logtype _type, QString _message);
    void log(QString _message);

	void addEmptyObject(DataType _type, int& _id);

	void addToolbox(QString name, QWidget *widget, QIcon *icon);

///////////////////////////////////////////////////////////////////////////////
// Slots
///////////////////////////////////////////////////////////////////////////////
public slots:

	QString version() { return QString("1.0.0"); };

private slots:

	void initializePlugin() override;

	// voronoi meshing functions
	void callVoronoi();
	void callPartition();
	void callDualStep();
	void callDual();
	void callFitting();
	void callSmooth();

	void testAlgorithm();

	// decimation meshing functions
	void callDecimationInit();
	void callDecimation();
	void callDecimationStep();

};
