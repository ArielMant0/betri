//=============================================================================
//
//  BezierTriangleMesh Object
//
//=============================================================================

#define BEZIERTRIANGLEMESHOBJECT_C

//== INCLUDES =================================================================

#include <OpenFlipper/common/Types.hh>
#include "BezierTriangleMesh.hh"

//== DEFINES ==================================================================

//== TYPEDEFS =================================================================

//== CLASS DEFINITION =========================================================

/** Constructor for Plane Objects. This object class gets a Separator Node giving
*  the root node to which it should be connected. The plane is generated internally
*  and all nodes for visualization will be added below the scenegraph node.\n
*  You dont need to create an object of this type manually. Use
*  the functions from LoadSaveInterface ( addEmpty() )
*/
BezierTriangleMeshObject::BezierTriangleMeshObject( ) :
  BaseObjectData(),
  btmeshNode_(NULL)
{
	setDataType(DATA_BEZIER_TRIANGLE_MESH);
	init();
}

//=============================================================================


/**
 * Copy Constructor - generates a copy of the given object
 */
BezierTriangleMeshObject::BezierTriangleMeshObject(const BezierTriangleMeshObject &_object) :
	BaseObjectData(_object)
{
    init(&_object.btmesh_);

    setName(name());
}

/** Destructor for BezierTriangleMesh Objects. The destructor deletes the Line and all
*  Scenegraph nodes associated with the Plane or the object.
*/
BezierTriangleMeshObject::~BezierTriangleMeshObject()
{
	// Delete the data attached to this object ( this will remove all perObject data)
	// Not the best way to do it but it will work.
	// This is only necessary if people use references to the plane below and
	// they do something with the plane in the destructor of their
	// perObjectData.
	deleteData();

	// No need to delete the scenegraph Nodes as this will be managed by baseplugin
	btmeshNode_ = NULL;
}

/** Cleanup Function for Plane Objects. Deletes the contents of the whole object and
* calls BezierTriangleMeshObject::init afterwards.
*/
void BezierTriangleMeshObject::cleanup() {

	BaseObjectData::cleanup();

	btmeshNode_ = NULL;

	setDataType(DATA_BEZIER_TRIANGLE_MESH);
	setTypeIcon(DATA_BEZIER_TRIANGLE_MESH,"PlaneType.png");

	init();
}

/**
 * Generate a copy
 */
BaseObject* BezierTriangleMeshObject::copy() {
    BezierTriangleMeshObject* object = new BezierTriangleMeshObject(*this);
    return dynamic_cast<BaseObject*>(object);
}

/** This function initializes the plane object. It creates the scenegraph nodes.
*/
void BezierTriangleMeshObject::init(const Plane* _btmesh) {

	if ( materialNode() == NULL) {
		std::cerr << "Error when creating BezierTriangleMesh Object! materialNode is NULL!" << std::endl;
		return;
	}

	btmeshNode_ = new BezierTriangleMeshNode(btmesh_, materialNode() , "NEW BezierTriangleMeshNode");

	if (_btmesh){
		btmesh_ = *_btmesh;
	} else {
		// TODO: !!
		/*btmesh_.setPlane( ACG::Vec3d(0.0, 0.0, 0.0), ACG::Vec3d(0.0, 1.0, 0.0) );
		btmesh_.setSize( 5.0, 5.0 );*/
	}

	materialNode()->set_point_size(3.0);
}

// ===============================================================================
// Name/Path Handling
// ===============================================================================

/** Set the name of an object. All Scenegraph nodes are renamed too. It also calls
* BaseObjectData::setName.
*/
void BezierTriangleMeshObject::setName(QString _name) {
	BaseObjectData::setName(_name);

	std::string nodename = std::string("BezierTriangleMeshNode for BezierTriangle "     + _name.toUtf8() );
	btmeshNode_->name( nodename );
}

// ===============================================================================
// Visualization
// ===============================================================================

BezierTriangleMeshNode* BezierTriangleMeshObject::BezierTriangleMeshNode() {
  return btmeshNode_;
}

// ===============================================================================
// Object information
// ===============================================================================

/** Returns a string containing all information about the current object. This also
* includes the information provided by BaseObjectData::getObjectinfo
*
* @return String containing the object information
*/
QString BezierTriangleMeshObject::getObjectinfo() {
  QString output;

  output += "========================================================================\n";
  output += BaseObjectData::getObjectinfo();

  if ( dataType( DATA_PLANE ) )
    output += "Object Contains Plane : ";

  ACG::Vec3d pos = btmeshNode_->position();
  ACG::Vec3d nor = btmeshNode_->normal();

  output += " Position ( " + QString::number(pos[0]) + ", " + QString::number(pos[1]) + ", " + QString::number(pos[2]) + ")";
  output += " Normal ( " + QString::number(nor[0]) + ", " + QString::number(nor[1]) + ", " + QString::number(nor[2]) + ")";

  output += "========================================================================\n";
  return output;
}

// ===============================================================================
// Content
// ===============================================================================

void BezierTriangleMeshObject::update(UpdateType _type) {
  if (btmeshNode_)
    btmeshNode_->update();
}

// ===============================================================================
// Data
// ===============================================================================

Plane& BezierTriangleMeshObject::plane() {
  return btmesh_;
}

void BezierTriangleMeshObject::plane(Plane _plane) {
  btmesh_ = _plane;
}

// ===============================================================================
// Picking
// ===============================================================================

/** Given an node index from PluginFunctions::scenegraphPick this function can be used to
* check if the BezierTriangleMeshNode of the object has been picked.
*
* @param _node_idx Index of the picked plane node
* @return bool if the BezierTriangleMeshNode of this object is the picking target.
*/
bool BezierTriangleMeshObject::picked( uint _node_idx ) {
  return ( _node_idx == btmeshNode_->id() );
}

void BezierTriangleMeshObject::enablePicking( bool _enable ) {
  btmeshNode_->enablePicking( _enable );
}

bool BezierTriangleMeshObject::pickingEnabled() {
  return btmeshNode_->pickingEnabled();
}

//=============================================================================

