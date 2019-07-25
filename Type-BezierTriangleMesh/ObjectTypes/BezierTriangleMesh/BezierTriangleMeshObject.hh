#pragma once

//=============================================================================
//
//  Types
//
//=============================================================================

/**
 * \file BezierTriangleMeshObject.hh
 * This File contains the BezierTriangleMesh Object
 */


//== INCLUDES =================================================================

#include <OpenFlipper/common/BaseObjectData.hh>

#include <OpenFlipper/common/ObjectTypeDLLDefines.hh>

#include <Type-BezierTriangleMesh/BezierTriangleMeshType.hh>
#include "BezierTriangleMeshNode.hh"


//== TYPEDEFS =================================================================

// //== CLASS DEFINITION =========================================================

class OBJECTTYPEDLLEXPORT BezierTriangleMeshObject : public BaseObjectData
{
	using BTMesh = BezierTriangleMesh;

	friend class TypeBezierTriangleMeshPlugin;

public:
    /// constructor
	BezierTriangleMeshObject();

    /** \brief copy constructor
     *
     *  Create a copy of this object
     */
	BezierTriangleMeshObject(const BezierTriangleMeshObject& _object);

    /// destructor
    virtual ~PlaneObject();

    /// Reset current object, including all related nodes.
    virtual void cleanup();

    /** return a full copy of this object ( All scenegraph nodes will be created )
     *  but the object will not be a part of the object tree.
     */
    BaseObject* copy();


protected:
	/// Initialize current object, including all related nodes.
    virtual void init(const BTMesh* _btmesh=0);

//===========================================================================
/** @name Name and Path handling
* @{ */
//===========================================================================
public:
    /// Set the name of the Object
    void setName(QString _name);

//===========================================================================
/** @name Data
* @{ */
//===========================================================================
public:
	BTMesh& btmesh();
	void btmesh(BTMesh _btmesh);

private:
	  BTMesh btmesh_;

  /** @} */

  //===========================================================================
  /** @name Update handling
   *
   *  This is mostly private. Updates have to be triggered via
   *  emit updatedObject()
   *
   * @{ */
  //===========================================================================

protected:
    /** \brief  This function is called to update the object
     *
     * If the object changes, the core will call this function. This function
     * triggers an vbo update in the plane node.
     *
     * \note Do not call this function yourself to avoid unnecessary overhead(the core will call it when it is required)
     */
    void update(UpdateType _type=UPDATE_ALL );

   /** @} */

//===========================================================================
/** @name Visualization
* @{ */
//===========================================================================
public:
    /// Get the scenegraph Node
    BezierTriangleMeshNode *btmeshNode();

private:
	BezierTriangleMeshNode *btmeshNode_;

  /** @} */


//===========================================================================
/** @name Object Information
* @{ */
//===========================================================================
public:
    /// Get all Info for the Object as a string
    QString getObjectinfo();

  /** @} */

//===========================================================================
/** @name Picking
* @{ */
//===========================================================================
public:
    /// detect if the node has been picked
    bool picked(uint _node_idx);

    /// Enable or disable picking for this Object
    void enablePicking(bool _enable);

    /// Check if picking is enabled for this Object
    bool pickingEnabled();

  /** @} */

};
