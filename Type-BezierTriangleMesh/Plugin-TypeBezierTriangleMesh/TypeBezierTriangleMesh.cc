#include "TypeBezierTriangleMesh.hh"

#include <OpenFlipper/common/BackupData.hh>
#include "BezierTMeshBackup.hh"
#include <OpenFlipper/common/GlobalOptions.hh>

TypeBezierTriangleMeshPlugin::TypeBezierTriangleMeshPlugin() {}

bool TypeBezierTriangleMeshPlugin::registerType()
{
	addDataType("BezierTriangleMesh",tr("BezierTriangleMesh"));
	setTypeIcon("BezierTriangleMesh", "beziertriangle.jpg");
	return true;
}

int TypeBezierTriangleMeshPlugin::addEmpty()
{
	// new object data struct
	BTMeshObject *object = new BTMeshObject(typeId("BezierTriangleMesh"));

	if (OpenFlipperSettings().value("Core/File/AllTarget", false).toBool()) {
		object->target(true);
	} else {
		// only the first object in the scene will be target
		if (PluginFunctions::objectCount() == 1)
			object->target(true);
		// if no target is available, we set the new object as target
		if (PluginFunctions::targetCount() == 0)
			object->target(true);
	}

	QString name = QString(tr("BezierTriangleMesh_%1.off").arg(object->id()));

	// call the local function to update names
	QFileInfo f(name);
	object->setName(f.fileName());

	// set the default colors
	const QColor color = OpenFlipper::Options::defaultColor();
	const ACG::Vec4f default_color(color.redF(), color.greenF(), color.blueF(), color.alphaF());
	object->materialNode()->set_color(default_color);

	object->update();

	object->show();

	emit emptyObjectAdded(object->id());

	return object->id();
}


void TypeBezierTriangleMeshPlugin::generateBackup(int _id, QString _name, UpdateType _type)
{
	BaseObjectData* object = 0;
	PluginFunctions::getObject(_id, object);

	BTMeshObject* meshObj = PluginFunctions::btMeshObject(object);

	if (meshObj != 0) {
		//get backup object data
		BackupData* backupData = 0;

		if (object->hasObjectData(OBJECT_BACKUPS)) {
			backupData = dynamic_cast<BackupData*>(object->objectData(OBJECT_BACKUPS));
		} else {
			//add backup data
			backupData = new BackupData(object);
			object->setObjectData(OBJECT_BACKUPS, backupData);
		}

		//store a new backup
		BezierTMeshBackup *backup = new BezierTMeshBackup(meshObj, _name, _type);

		backupData->storeBackup(backup);
	}
}
