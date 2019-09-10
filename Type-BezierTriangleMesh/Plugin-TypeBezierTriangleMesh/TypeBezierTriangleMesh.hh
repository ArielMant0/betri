#pragma once

#include <QObject>

#include <OpenFlipper/common/Types.hh>
#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/LoadSaveInterface.hh>
#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/BasePlugin/TypeInterface.hh>
#include <OpenFlipper/BasePlugin/ContextMenuInterface.hh>

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

class TypeBezierTriangleMeshPlugin : public QObject,
	BaseInterface,
	LoadSaveInterface,
	LoggingInterface,
	TypeInterface,
	ContextMenuInterface
{
	Q_OBJECT
	Q_INTERFACES(BaseInterface)
	Q_INTERFACES(LoadSaveInterface)
	Q_INTERFACES(LoggingInterface)
	Q_INTERFACES(TypeInterface)
	Q_INTERFACES(ContextMenuInterface)

	Q_PLUGIN_METADATA(IID "org.OpenFlipper.Plugins.Plugin-TypeBezierTriangleMeshPlugin")

signals:
    // Logging interface
    void log(Logtype _type, QString _message);
    void log(QString _message);

	// ContextMenuInterface
	void addContextMenuItem(QAction* _action, ContextMenuType _type);
	void addContextMenuItem(QAction* _action, DataType _objectType, ContextMenuType _type);

    // LoadSave Interface
    void emptyObjectAdded(int _id);

private slots: // interface slots

	// BaseInterface
	void pluginsInitialized();
    void noguiSupported() {}

	// ContextMenuInterface
	void slotUpdateContextMenu(int _objectId);

private slots: // own slots

	/// Slot triggered from context menu, if the control net should be rendered
	void slotRenderControlNet();

	/// Slot triggered from context menu, if the surface should be rendered
	void slotRenderSurface();

	/// Slot triggered from context menu, if the selection rendering should be altered
	void slotRenderSelection(QAction* _action);

public:

	 TypeBezierTriangleMeshPlugin();
     ~TypeBezierTriangleMeshPlugin() {}

     QString name() { return (QString("TypeBezierTriangleMeshPlugin")); }
     QString description() { return (QString(tr("Register BezierTriangleMesh type"))); }

     bool registerType();

    // Type Interface
    int addEmpty();
    DataType supportedType() { return DATA_BEZIER_TRIANGLE_MESH; }
	void generateBackup(int _id, QString _name, UpdateType _type);

public slots:

    // Base Interface
    QString version() { return QString("1.0"); }


private:

	/// Context menu action
	QAction* renderControlNetAction_;

	/// Context menu action
	QAction* renderSurfaceAction_;

	/// Context menu action (render selection texture)
	QAction* renderCPSelectionAction_;
	QAction* renderKnotSelectionAction_;
	QAction* renderNoSelectionAction_;
};
