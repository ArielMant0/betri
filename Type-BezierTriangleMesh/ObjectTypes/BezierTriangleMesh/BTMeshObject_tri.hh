#pragma once

//== INCLUDES =================================================================
#include "BezierTMesh.hh"

#ifndef DRAW_CURVED

// -------------------- OpenMesh
#include <OpenFlipper/common/ObjectTypeDLLDefines.hh>

#include <ObjectTypes/BTBaseMeshObject/BTBaseMeshObjectT.hh>

//== TYPEDEFS =================================================================

/// Texture Node
using TextureNode = ACG::SceneGraph::TextureNode;

//== CLASS DEFINITION =========================================================

//// Type for a MeshObject containing a bezier triangle mesh
class OBJECTTYPEDLLEXPORTONLY BTMeshObject : public BTBaseMeshObject<BezierTMesh> {

public:
	/** \brief copy constructor
	 *
	 *  Create a copy of this object
	 */
	BTMeshObject(const BTMeshObject &_object);

	/** \brief Constructor
	*
	* This is the standard constructor for MeshObjects. As triangle and Poly Meshes are handled by this class, the
	* typeId is passed to the MeshObject to specify it.
	*
	* @param _typeId   This is the type Id the Object will use. Should be typeId("TriangleMesh") or typeId("PolyMesh")
	*/
	BTMeshObject();

	/// destructor
	virtual ~BTMeshObject();

	/** return a full copy of this object ( All scenegraph nodes will be created )
	 *  but the object will not be a part of the object tree.
	 */
	BaseObject* copy();

public:
	/// Refine picking on triangle meshes
	ACG::Vec3d refinePick(
		ACG::SceneGraph::PickTarget _pickTarget,
		const ACG::Vec3d _hitPoint,
		const ACG::Vec3d _start,
		const ACG::Vec3d _dir,
		const unsigned int _targetIdx
	);

};
#endif

//=============================================================================
