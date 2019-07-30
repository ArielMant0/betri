#pragma once

#include <QObject>

#include <OpenFlipper/common/Types.hh>
#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/LoadSaveInterface.hh>
#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/BasePlugin/TypeInterface.hh>

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

class TypeBezierTriangleMeshPlugin : public QObject,
	BaseInterface,
	LoadSaveInterface,
	LoggingInterface,
	TypeInterface
{
	Q_OBJECT
	Q_INTERFACES(BaseInterface)
	Q_INTERFACES(LoadSaveInterface)
	Q_INTERFACES(LoggingInterface)
	Q_INTERFACES(TypeInterface)

	Q_PLUGIN_METADATA(IID "org.OpenFlipper.Plugins.Plugin-TypeBezierTriangleMeshPlugin")

signals:
    // Logging interface
    void log(Logtype _type, QString _message);
    void log(QString _message);

    // LoadSave Interface
    void emptyObjectAdded(int _id);

private slots:

    void noguiSupported() {} ;

public :

     ~TypeBezierTriangleMeshPlugin() {};
	 TypeBezierTriangleMeshPlugin();

     QString name() { return (QString("TypeBezierTriangleMeshPlugin")); };
     QString description() { return (QString(tr("Register BezierTriangleMesh type"))); };

     bool registerType();

public slots:

    // Base Interface
    QString version() { return QString("1.0"); };

    // Type Interface
    int addEmpty();
    DataType supportedType() { return DATA_BEZIER_TRIANGLE_MESH; };

	// Type Interface
	void generateBackup(int _id, QString _name, UpdateType _type);

};
