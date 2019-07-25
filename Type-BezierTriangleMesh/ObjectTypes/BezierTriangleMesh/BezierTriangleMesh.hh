#pragma once
/**
 * \file BezierTriangleMesh.hh
 * This File contains all required includes for using BezierTriangleMeshes
*/

#define DATA_BEZIER_TRIANGLE_MESH typeId("BezierTriangleMesh")

//== INCLUDES =================================================================

#include <ObjectTypes/MeshObject/MeshObjectT.hh>
#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMeshTypes.hh>
#include <OpenFlipper/common/ObjectTypeDLLDefines.hh>

 /// Type for a MeshObject containing a bezier triangle mesh
class OBJECTTYPEDLLEXPORTONLY BTMeshObject : public MeshObject<BezierTMesh> {

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
	BTMeshObject(DataType _typeId);

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

#include <ObjectTypes/BezierTriangleMesh/PluginFunctionsBezierTriangleMesh.hh>

