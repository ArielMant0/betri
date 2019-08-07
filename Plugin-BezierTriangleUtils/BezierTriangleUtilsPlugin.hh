#pragma once

#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/ToolboxInterface.hh>
#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/BasePlugin/RenderInterface.hh>

#include <OpenFlipper/common/Types.hh>

#include <ObjectTypes/BezierTriangleMesh/globals/BezierOptions.hh>

#include <qlineedit.h> // TODO

class BezierTriangleUtilsPlugin : public QObject,
	BaseInterface,
	ToolboxInterface,
	LoggingInterface
{
	Q_OBJECT
	Q_INTERFACES(BaseInterface)
	Q_INTERFACES(ToolboxInterface)
	Q_INTERFACES(LoggingInterface)

	Q_PLUGIN_METADATA(IID "org.OpenFlipper.Plugins.Plugin-BezierTriangleUtils")

public:

	BezierTriangleUtilsPlugin() : m_tool(0) {}

	~BezierTriangleUtilsPlugin() {};

	QString name() { return QString("BezierTriangleUtilsPlugin"); };

	QString description()
	{
		return QString("Adds a Bezier Triangle Mesh data structure and some matching algorithms");
	};

private:

	QWidget *m_tool;
	QLineEdit *echoLineEdit; // TODO

	void switchViewMode();

///////////////////////////////////////////////////////////////////////////////
// Signals
///////////////////////////////////////////////////////////////////////////////
signals:

	void updateView();
    void updatedObject(int _identifier, const UpdateType& _type);

	void log(Logtype _type, QString _message);
    void log(QString _message);

	void addToolbox(QString name, QWidget *widget, QIcon *icon);

///////////////////////////////////////////////////////////////////////////////
// Slots
///////////////////////////////////////////////////////////////////////////////
public slots:

	QString version() { return QString("1.0.0"); };

private slots:

	void echoChanged(int);

	void initializePlugin() override;


	void convertMesh();
	void tessellateMesh();

	// option setting function
	void setTessAmount(int);
	void setTessType(int);

	void callVoronoi();

};
