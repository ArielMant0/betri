#pragma once

#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/ToolboxInterface.hh>
#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/BasePlugin/RenderInterface.hh>
#include <OpenFlipper/BasePlugin/LoadSaveInterface.hh>

#include <OpenFlipper/common/Types.hh>


#include <qlineedit.h> // TODO
#include <qpushbutton.h> // TODO

class BezierTriangleUtilsPlugin : public QObject,
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

	Q_PLUGIN_METADATA(IID "org.OpenFlipper.Plugins.Plugin-BezierTriangleUtils")

public:

	BezierTriangleUtilsPlugin() : m_tool(0), m_steps(), m_target() {}

	~BezierTriangleUtilsPlugin() {};

	QString name() { return QString("BezierTriangleUtilsPlugin"); };

	QString description()
	{
		return QString("Adds a Bezier Triangle Mesh data structure and some matching algorithms");
	};

private:

	QWidget *m_tool;
	QLineEdit *echoLineEdit; // TODO
	std::vector<QPushButton*> m_voronoiSteps;

	std::map<int, int> m_steps, m_target;

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

	QString version() { return QString("1.0.0");
	};

private slots:

	void initializePlugin() override;

	void tessellateMesh();

	// option setting function
	void setTessType(int);
	void setTessAmount(int);
	void setTessMode(int);
	void setBoundVType(int);
	void setVisulisationType(int);

	// TODO wirklich nötig?
	void setBError(int);
	void setDError(int);
	void setNewtonIt(int);

	void setCulling(bool);
	void setBoundVShow(bool);

	// voronoi meshing functions
	void callVoronoi();
	void callPartition();
	void callDualStep();
	void callDual();
	void callFitting();
	void callSmooth();

	void testAlgorithm();

	// decimation meshing functions
	void callDecimation();
	void callDecimationStep();

};
