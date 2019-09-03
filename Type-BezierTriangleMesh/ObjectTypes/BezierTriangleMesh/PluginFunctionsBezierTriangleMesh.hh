#pragma once

//=============================================================================
//
//  Standard Functions
//
//=============================================================================

#include <OpenFlipper/common/Types.hh>
#include <ObjectTypes/BezierTriangleMesh/globals/BezierOptions.hh>
//#include <OpenFlipper/common/ObjectTypeDLLDefines.hh>

/** The Namespace PluginFunctions contains functions for all plugins. These functions should be used to get the
 *  objects to work on or to set modes in the examiner widget. */
namespace PluginFunctions {

OBJECTTYPEDLLEXPORT
void betriOption(betri::BezierOption option, int value);

OBJECTTYPEDLLEXPORT
int betriOption(betri::BezierOption option);

//=======================================
// Get Source/Target objects
/** @name Active Objects
* @{ */
//=======================================

/** \brief Get a pointer to every Triangle Mesh which is marked as a source mesh.
 *
 * @param _meshes ( vector returning the source meshes )
 * @return false, if no mesh is selected as source
*/
OBJECTTYPEDLLEXPORT
bool getSourceMeshes(std::vector<BezierTMesh*> &_meshes);

/** \brief Get a pointer to every Triangle Mesh which is marked as a target mesh.
 *
 * @param _meshes ( vector returning the target meshes )
 * @return false, if no mesh is selected as target
*/
OBJECTTYPEDLLEXPORT
bool getTargetMeshes(std::vector<BezierTMesh*> &_meshes);

/** @} */

//=======================================
// Get Objects by their identifier
	/** @name Identifier handling
	* @{ */
	//=======================================

	/** This functions returns the object with the given id if it is a TriMeshObject.
	 * See get_object(  int _identifier , BaseObject*& _object ) for more details.
	 */
OBJECTTYPEDLLEXPORT
bool getObject(int _identifier, BTMeshObject *&_object);


/** \brief Get the Triangle Mesh which has the given identifier.
 *
 *   Every loaded object has a unique identifier which is stored in the id field of the object container.
 *   Use this function to get the mesh which has this id. This can be used for a consistent mapping
 *   even if the data objects change during plugin operations (e.g. selection and main algorithm).\n
 *   This function checks, if the object requested contains a mesh.
 * @param _identifier Object id to search for
 * @param _mesh  returns the mesh
 * @return Mesh found?
 */
OBJECTTYPEDLLEXPORT
bool getMesh(int _identifier, BezierTMesh *& _mesh);

/** @} */

//=======================================
	/** @name Getting data from objects and casting between them
	 * @{ */
	 //=======================================

	 /** \brief Get a triangle mesh from an object.
	  *
	  * @param _object The object should be of type BaseDataObject. If the content is a triangle Mesh, a
	  *                triangle mesh will be returned. Otherwise a NULL pointer is returned.
	  */
OBJECTTYPEDLLEXPORT
BezierTMesh* btMesh(BaseObjectData* _object);

/** \brief Get a triangle mesh from an object id.
 *
 * @param _identifier Identifier of the object. If its a triangle mesh, the function will return the pointer to the mesh
 *                    otherwise 0
 */
OBJECTTYPEDLLEXPORT
BezierTMesh* btMesh(int _identifier);

/** \brief Cast an BaseObject to a TriMeshObject if possible
 *
 * @param _object The object should be of type BaseDataObject. If the content is a triangle Mesh, a
 *                a TriMeshObject is returned. Otherwise a NULL pointer is returned.
 */
OBJECTTYPEDLLEXPORT
BTMeshObject* btMeshObject(BaseObjectData* _object);

/** \brief Get an TriMeshObject from the given id If possible
*
* @param _objectId Id of the requested Object.
* @return If the content is a volume, a pointer to the TriMeshObject is returned, otherwise a NULL pointer.
*/
OBJECTTYPEDLLEXPORT
BTMeshObject* btMeshObject(int _objectId);

/** @} */
}