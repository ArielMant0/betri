#include "TypeBezierTriangleMesh.hh"

#include "BezierTMeshBackup.hh"
#include <OpenFlipper/BasePlugin/PluginFunctions.hh>
#include <OpenFlipper/common/BackupData.hh>
#include <OpenFlipper/common/GlobalOptions.hh>

#include <QMenu>

TypeBezierTriangleMeshPlugin::TypeBezierTriangleMeshPlugin() :
	renderControlNetAction_(0),
	renderSurfaceAction_(0),
	renderCPSelectionAction_(0),
	renderKnotSelectionAction_(0),
	renderNoSelectionAction_(0) {}

void TypeBezierTriangleMeshPlugin::pluginsInitialized()
{

	if (OpenFlipper::Options::gui()) {

		QMenu* contextMenu = new QMenu("Rendering");

		QString iconPath = OpenFlipper::Options::iconDirStr() + OpenFlipper::Options::dirSeparator();

		// Render Control Net
		renderControlNetAction_ = new QAction(tr("Render Control Net"), this);
		renderControlNetAction_->setStatusTip(tr("Render Control Net"));
		//    renderControlNetAction_->setIcon( QIcon(iconPath + "showIndices.png") );
		renderControlNetAction_->setCheckable(true);
		renderControlNetAction_->setChecked(false);

		// Render Surface
		renderSurfaceAction_ = new QAction(tr("Render Surface"), this);
		renderSurfaceAction_->setStatusTip(tr("Render Surface"));
		//    renderSurfaceAction_->setIcon( QIcon(iconPath + "coordsys.png") );
		renderSurfaceAction_->setCheckable(true);
		renderSurfaceAction_->setChecked(true);

		QActionGroup* group = new QActionGroup(this);
		group->setExclusive(true);

		// Render Control Point Selection
		renderCPSelectionAction_ = new QAction(tr("Render Control Point Selection"), group);
		renderCPSelectionAction_->setStatusTip(tr("Render Control Point Selection"));
		//    renderCPSelectionAction_->setIcon( QIcon(iconPath + "coordsys.png") );
		renderCPSelectionAction_->setCheckable(true);
		renderCPSelectionAction_->setChecked(true);

		// Render Knot Selection
		renderKnotSelectionAction_ = new QAction(tr("Render Knot Selection"), group);
		renderKnotSelectionAction_->setStatusTip(tr("Render Knot Selection"));
		//    renderKnotSelectionAction_->setIcon( QIcon(iconPath + "coordsys.png") );
		renderKnotSelectionAction_->setCheckable(true);
		renderKnotSelectionAction_->setChecked(true);

		// Render No Selection
		renderNoSelectionAction_ = new QAction(tr("Don't Render Selection"), group);
		renderNoSelectionAction_->setStatusTip(tr("Don't Render Selection"));
		//    renderNoSelectionAction_->setIcon( QIcon(iconPath + "coordsys.png") );
		renderNoSelectionAction_->setCheckable(true);
		renderNoSelectionAction_->setChecked(true);


		connect(renderControlNetAction_, SIGNAL(triggered()), this, SLOT(slotRenderControlNet()));
		connect(renderSurfaceAction_, SIGNAL(triggered()), this, SLOT(slotRenderSurface()));

		connect(group, SIGNAL(triggered(QAction*)), this, SLOT(slotRenderSelection(QAction*)));

		contextMenu->addAction(renderControlNetAction_);
		contextMenu->addAction(renderSurfaceAction_);
		contextMenu->addSeparator();
		contextMenu->addAction(renderCPSelectionAction_);
		contextMenu->addAction(renderKnotSelectionAction_);
		contextMenu->addAction(renderNoSelectionAction_);

		emit addContextMenuItem(contextMenu->menuAction(), DATA_BEZIER_TRIANGLE_MESH, CONTEXTOBJECTMENU);
	}
}

void TypeBezierTriangleMeshPlugin::slotUpdateContextMenu(int _objectId)
{
	if (_objectId == -1)
		return;

	BaseObjectData* object;
	if (!PluginFunctions::getObject(_objectId, object))
		return;

	BTMeshObject* surfaceObject = dynamic_cast<BTMeshObject*>(object);

	if (surfaceObject != 0) {
		renderControlNetAction_->setChecked(surfaceObject->bezierTriangleMeshNode()->render_control_net());
		renderSurfaceAction_->setChecked(surfaceObject->bezierTriangleMeshNode()->render_bspline_surface());

		/*
		renderCPSelectionAction_->setChecked(surfaceObject->splineSurfaceNode()->get_selection_draw_mode() == ACG::SceneGraph::BSplineSurfaceNodeT<BSplineSurface>::CONTROLPOINT);
		renderKnotSelectionAction_->setChecked(surfaceObject->splineSurfaceNode()->get_selection_draw_mode() == ACG::SceneGraph::BSplineSurfaceNodeT<BSplineSurface>::KNOTVECTOR);
		renderNoSelectionAction_->setChecked(surfaceObject->splineSurfaceNode()->get_selection_draw_mode() == ACG::SceneGraph::BSplineSurfaceNodeT<BSplineSurface>::NONE);
		*/
	}
}

void TypeBezierTriangleMeshPlugin::slotRenderControlNet() {

	QVariant contextObject = renderControlNetAction_->data();
	int objectId = contextObject.toInt();

	if (objectId == -1)
		return;

	BaseObjectData* object;
	if (!PluginFunctions::getObject(objectId, object))
		return;

	BTMeshObject* surfaceObject = dynamic_cast<BTMeshObject*>(object);

	if (surfaceObject != 0) {
		surfaceObject->bezierTriangleMeshNode()->render_control_net(renderControlNetAction_->isChecked());
		emit updatedObject(objectId, UPDATE_ALL);
	}
}

//-----------------------------------------------------------------------------

void TypeBezierTriangleMeshPlugin::slotRenderSurface() {

	QVariant contextObject = renderSurfaceAction_->data();
	int objectId = contextObject.toInt();

	if (objectId == -1)
		return;

	BaseObjectData* object;
	if (!PluginFunctions::getObject(objectId, object))
		return;

	BTMeshObject* surfaceObject = dynamic_cast<BTMeshObject*>(object);

	if (surfaceObject != 0) {
		surfaceObject->bezierTriangleMeshNode()->render_bspline_surface(renderSurfaceAction_->isChecked());
		emit updatedObject(objectId, UPDATE_ALL);
	}
}

//-----------------------------------------------------------------------------

void TypeBezierTriangleMeshPlugin::slotRenderSelection(QAction* _action) {

	QVariant contextObject = _action->data();
	int objectId = contextObject.toInt();

	if (objectId == -1)
		return;

	BaseObjectData* object;
	if (!PluginFunctions::getObject(objectId, object))
		return;

	BTMeshObject* surfaceObject = dynamic_cast<BTMeshObject*>(object);

	if (surfaceObject != 0) {
		/*if (_action == renderCPSelectionAction_) {
			surfaceObject->bezierTriangleMeshNode()->set_selection_draw_mode(ACG::SceneGraph::BezierTriangleMeshNode<BezierTMesh>::CONTROLPOINT);
			emit updatedObject(objectId, UPDATE_ALL);
		}
		else if (_action == renderKnotSelectionAction_) {
			surfaceObject->bezierTriangleMeshNode()->set_selection_draw_mode(ACG::SceneGraph::BezierTriangleMeshNode<BezierTMesh>::KNOTVECTOR);
			emit updatedObject(objectId, UPDATE_ALL);
		}
		else if (_action == renderNoSelectionAction_) {
			surfaceObject->bezierTriangleMeshNode()->set_selection_draw_mode(ACG::SceneGraph::BezierTriangleMeshNode<BezierTMesh>::NONE);
			emit updatedObject(objectId, UPDATE_ALL);
		}*/
	}
}

bool TypeBezierTriangleMeshPlugin::registerType()
{
	addDataType("BezierTriangleMesh", tr("BezierTriangleMesh"));
	setTypeIcon("BezierTriangleMesh", "beziertriangle.jpg");
	return true;
}

int TypeBezierTriangleMeshPlugin::addEmpty()
{
	// new object data struct
	BTMeshObject *object = new BTMeshObject();
	object->setDataType(DATA_BEZIER_TRIANGLE_MESH);

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
