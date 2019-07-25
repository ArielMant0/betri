#pragma once

#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/ToolboxInterface.hh>
#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/BasePlugin/RenderInterface.hh>

#include <OpenFlipper/common/Types.hh>

#include "BezierTriangleMesh.hh"

class BezierTrianglePlugin : public QObject,
	BaseInterface, ToolboxInterface, LoggingInterface//, RenderInterface
{
	Q_OBJECT
	Q_INTERFACES(BaseInterface)
	Q_INTERFACES(ToolboxInterface)
	Q_INTERFACES(LoggingInterface)
	//Q_INTERFACES(RenderInterface)

	Q_PLUGIN_METADATA(IID "org.OpenFlipper.Plugins.Plugin-BezierTriangle")

public:

	BezierTrianglePlugin() : m_tool(0) {}

	~BezierTrianglePlugin() {};

	QString name() { return QString("BezierTrianglePlugin"); };

	QString description()
	{
		return QString("Adds a Bezier Triangle Mesh data structure and some matching algorithms");
	};

private:

	BezierTMesh m_mesh;
	QWidget *m_tool;

	void switchViewMode();

	void createBezierMesh(TriMesh *mesh);

signals:

	void updateView();
    void updatedObject(int _identifier, const UpdateType& _type);

	void log(Logtype _type, QString _message);
    void log(QString _message);

	void addToolbox(QString name, QWidget *widget, QIcon *icon);

public slots:

	QString version() { return QString("1.0.0"); };

	//void render(ACG::GLState *_glState, Viewer::ViewerProperties &_properties) override;

private slots:

	void initializePlugin() override;

	void pluginsInitialized() override;

	/*void supportedDrawModes(ACG::SceneGraph::DrawModes::DrawMode &_mode) override
	{
		_mode = ACG::SceneGraph::DrawModes::DEFAULT;
	}

	QString rendererName() override
	{
		return QString("Bezier Triangle Mesh Renderer");
	}*/

	void loadMesh();

};
