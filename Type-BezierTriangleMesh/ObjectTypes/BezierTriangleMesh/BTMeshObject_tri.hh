#pragma once

//== INCLUDES =================================================================
#include "BezierTMesh.hh"

// -------------------- OpenMesh
// #include <OpenMesh/Core/IO/MeshIO.hh>

// #include <ACG/Scenegraph/SeparatorNode.hh>
// #include <ACG/Scenegraph/EnvMapNode.hh>
// #include <ACG/Scenegraph/ShaderNode.hh>
// #include <ACG/Scenegraph/StatusNodesT.hh>

// #include <OpenFlipper/common/GlobalDefines.hh>

// #include <ObjectTypes/BezierTriangleMesh/BTStatusNodeMods.hh>
// #include <ObjectTypes/BezierTriangleMesh/BTStatusViewNodeT.hh>
// #include <OpenFlipper/common/BaseObjectData.hh>
#include <OpenFlipper/common/ObjectTypeDLLDefines.hh>

// -------------------- BSP
// #include <ACG/Geometry/bsp/TriangleBSPT.hh>


//#include <OpenFlipper/common/BaseObjectData.hh>
// #include <ACG/Scenegraph/MeshNode2T.hh>
#include <ObjectTypes/BTBaseMeshObject/BTBaseMeshObjectT.hh>



// #include "BezierTriangleMeshNode.hh"

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

//=============================================================================
