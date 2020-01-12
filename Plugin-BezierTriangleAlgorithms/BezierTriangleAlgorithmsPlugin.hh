#pragma once

#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/ToolboxInterface.hh>
#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/BasePlugin/RenderInterface.hh>
#include <OpenFlipper/BasePlugin/LoadSaveInterface.hh>

#include <OpenFlipper/common/Types.hh>

#include <qlineedit.h>
#include <qpushbutton.h>
#include <qcheckbox.h>
#include <qcombobox.h>

#include <set>

#include "AlgorithmTimer.hh"

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

	BezierTriangleAlgorithmsPlugin() : m_tool(0), m_useTimer(false) {}

	~BezierTriangleAlgorithmsPlugin() {};

	QString name() { return QString("BezierTriangleAlgorithmsPlugin"); };

	QString description()
	{
		return QString("Construct Bezier triangle meshes from triangle meshes");
	};

private:

	bool applyTargetDegree(BaseObject *meshObj);

private:

	QWidget *m_tool;
	// TODO: disable other buttons
	std::vector<QPushButton*> m_voronoiSteps;
	std::array<QCheckBox*, 4> m_vFlags;
	QComboBox *m_vparam;
	std::array<QCheckBox*, 2> m_dFlags;

	// whether sth started ... TODO
	std::set<int> m_vinit;

	bool m_useTimer;
	AlgorithmTimer m_timer;

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
	void deleteObject(int _id);

	void addToolbox(QString name, QWidget *widget, QIcon *icon);

///////////////////////////////////////////////////////////////////////////////
// Slots
///////////////////////////////////////////////////////////////////////////////
public slots:

	QString version() { return QString("1.0.0"); };

private slots:

	void initializePlugin() override;

	void setTargetDegree(int degree);

	// voronoi meshing functions
	void callVoronoi();
	void callPartition();
	void callPartitionStep();
	void callDualStep();
	void callDual();
	void callFitting();

	void testAlgorithm();

	// decimation meshing functions
	void callDecimationInit();
	void callDecimation();
	void callDecimationStep();
};
