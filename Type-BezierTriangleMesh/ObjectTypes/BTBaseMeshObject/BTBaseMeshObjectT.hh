#pragma once

//== INCLUDES =================================================================

// -------------------- OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>

#include <ACG/Scenegraph/SeparatorNode.hh>
#include <ACG/Scenegraph/EnvMapNode.hh>
#include <ACG/Scenegraph/ShaderNode.hh>
#include <ACG/Scenegraph/StatusNodesT.hh>

#include <OpenFlipper/common/GlobalDefines.hh>

#include <ObjectTypes/BTBaseMeshObject/BTStatusNodeMods.hh>
#include <ObjectTypes/BTBaseMeshObject/BTStatusViewNodeT.hh>
#include <OpenFlipper/common/BaseObjectData.hh>

// -------------------- BSP
#include <ACG/Geometry/bsp/TriangleBSPT.hh>


#include <OpenFlipper/common/BaseObjectData.hh>
#include <ACG/Scenegraph/MeshNode2T.hh>



//== TYPEDEFS =================================================================

/// Texture Node
using TextureNode = ACG::SceneGraph::TextureNode;

//== CLASS DEFINITION =========================================================

/** This class provides the functionality for all kind of meshes for the framework
 */
template <class MeshT>
class DLLEXPORTONLY BTBaseMeshObject : public BaseObjectData {

  friend class TypeBezierTriangleMeshPlugin;

  public:

    /** \brief copy constructor
     *
     *  Create a copy of this object
     */
    BTBaseMeshObject(const BTBaseMeshObject &_object);

    /** \brief Constructor
    *
    * This is the standard constructor for MeshObjects. As triangle and Poly Meshes are handled by this class, the
    * typeId is passed to the BTBaseMeshObject to specify it.
    *
    * @param _typeId   This is the type Id the Object will use. Should be typeId("TriangleMesh") or typeId("PolyMesh")
    */
    BTBaseMeshObject(DataType _typeId);

    /// destructor
    virtual ~BTBaseMeshObject();

	/// Reset current object, including all related nodes.
    virtual void cleanup();

  protected:

    /// Initialise current object, including all related nodes.
    virtual void init(MeshT* _mesh = 0);

  //===========================================================================
  /** @name Name and Path handling
   * @{ */
  //===========================================================================
  public:

    /// Set the name of the Object
    void setName(QString _name);

    /** @} */

  //===========================================================================
  /** @name Content
   * @{ */
  //===========================================================================

  public:
    /// return a pointer to the mesh
    MeshT* mesh();

    /// return a const pointer to the mesh
    const MeshT* mesh() const;

  private:
    /// pointer to the mesh
    MeshT *mesh_;

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

    /// Update the whole Object (Selection,Topology,...)
    virtual void update(UpdateType _type = UPDATE_ALL);

    /// Call this function to update the modeling regions
    void updateSelection();

    /// Call this function to update the modeling regions
    void updateModelingRegions();

    /// Update Feature Visualization Node
    void updateFeatures();

    /// Update Geometry of all data structures
    void updateGeometry();

    /// Update Colors of all data structures
    void updateColor();

    /// Update Topology of all data structures
    void updateTopology();

    /// Update Texture of all data structures
    void updateTexture();


  /** @} */

  //===========================================================================
  /** @name Visualization
   * @{ */
  //===========================================================================

  public:

    /// Get the TextureNode of the current mesh
    ACG::SceneGraph::EnvMapNode* textureNode();

    /// Return pointer to the shader node
    ACG::SceneGraph::ShaderNode* shaderNode();

    /// Hide or show the selection Node of the object
    void hideSelection(bool _hide);

    /// return if the selections are currently visible
    bool selectionVisible();

    /// Hide or show the feature Node of the object
    void hideFeatures(bool _hide);

    /// return if the feature Node of the object is currently visible
    bool featuresVisible( );

    /// Hide or show the area Nodes of the object
    void hideArea(StatusBits _bit, bool _hide);

    /** \brief Return if the selected areas are currently visible
     *
     * \note If the bits contain multiple area bits, than only if all are visible,
     *       this function will return true
     *
     *  @param _bit Status bit to check
     *  @return All visible?
     */
    bool areaVisible(StatusBits _bit);

    /// Get the Scenegraph Mesh Node
    ACG::SceneGraph::MeshNodeT<MeshT>* meshNode();

    /// Get the BoundingBox of this object
    void boundingBox(ACG::Vec3d& _bbMin , typename ACG::Vec3d& _bbMax);

    BaseNode* primaryNode();

    /// set color for selection
    void setSelectionColor(const ACG::Vec4f& _color);
    /// get color for selection. returns -1 vector, if handle node does not exists
    ACG::Vec4f selectionColor() const;

    /// set color for areas
    void setAreaColor(const ACG::Vec4f& _color);
    /// get color for areas. returns -1 vector, if handle node does not exists
    ACG::Vec4f areaColor() const;

    /// set color for features
    void setFeatureColor(const ACG::Vec4f& _color);
    /// get color for features. returns -1 vector, if handle node does not exists
    ACG::Vec4f featureColor() const;

    /// set color for handles
    void setHandleColor(const ACG::Vec4f& _color);
    /// get color for handles. returns -1 vector, if handle node does not exists
    ACG::Vec4f handleColor() const;

    /// Returns the status node (visualizing the selection) if available,
    /// nullptr otherwise.
    ACG::SceneGraph::SelectionNodeT<MeshT>*
    statusNode() { return statusNode_; }

    /// Returns the area selection node if available, nullptr otherwise.
    ACG::SceneGraph::StatusNodeT<MeshT, AreaNodeMod<MeshT>>*
    areaNode() { return areaNode_; }

    /// Returns the handle selection node if available, nullptr otherwise.
    ACG::SceneGraph::StatusNodeT<MeshT, HandleNodeMod<MeshT>>*
    handleNode() { return handleNode_; }

    /// Returns the feature selection node if available, nullptr otherwise.
    ACG::SceneGraph::StatusNodeT<MeshT, FeatureNodeMod<MeshT>>*
    featureNode() { return featureNode_; }

  private :
    /// Status Node for a mesh, visualizing the selection state of a mesh
    ACG::SceneGraph::SelectionNodeT<MeshT> *statusNode_;

    /// Area selection Vis
    ACG::SceneGraph::StatusNodeT<MeshT, AreaNodeMod<MeshT>> *areaNode_;

    /// Handle selection Vis
    ACG::SceneGraph::StatusNodeT<MeshT, HandleNodeMod<MeshT>> *handleNode_;

    /// Feature selection Vis
    ACG::SceneGraph::StatusNodeT<MeshT, FeatureNodeMod<MeshT>> *featureNode_;

    /// Scenegraph Mesh Node
    ACG::SceneGraph::MeshNodeT<MeshT> *meshNode_;

    /// Scenegraph TextureNode
    ACG::SceneGraph::EnvMapNode *textureNode_;

    /// Scenegraph ShaderNode
    ACG::SceneGraph::ShaderNode *shaderNode_;

    /// Scenegraph StatusNodeView
    ACG::SceneGraph::BTStatusViewNodeT<MeshT> *statusView_;

  /** @} */

  //===========================================================================
  /** @name Load/Save
   * @{ */
  //===========================================================================

  public:

    /// Load a mesh from the given file
    bool loadMesh(QString _filename);

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

    /// Enable or disable picking for this Node
    void enablePicking(bool _enable);

    /// Check if picking is enabled for this Node
    bool pickingEnabled();

  /** @} */

  //===========================================================================
  /** @name BSP Extension ( Implements a BSP for the mesh used for fast searches )
   * @{ */
  //===========================================================================
  public:

    typedef OpenMeshTriangleBSPT<MeshT> OMTriangleBSP;

    /** Get a bsp for this object. Only supported for Triangle Meshes at the moment.
     *
     *
     * @return Pointer to bsp or Null if unsupported for this type.
     */
    OMTriangleBSP* requestTriangleBsp();

    /**  If something in the mesh changes, call this function to clear the octree.
     *  You have to do a new request as the old one will be deleted.
     * @todo : Update the tree when the update function of this object is called.
     * @todo : dont recreate but update the old one.
     * @return The new pointer to the bsp or Null if unsupported
     */
     void invalidateTriangleBsp();


    /** Update the bsp for this object. Only supported for Triangle Meshes at the moment.
     *
     *
     * @return Pointer to bsp or Null if unsupported for this type.
     */
     OMTriangleBSP* resetTriangleBsp();

     /** \brief check if a BSP has been computed and is valid
     *
     * This function checks if a bsp has been computed for this mesh object and
     * if it is still valid (meaning, nothing has been changed on the object after
     * the BSP has been computed
     *
     * @return BSP valid?
     */
     bool hasBsp() const;


  private :
    /// If requested a bsp is created for this object
    OMTriangleBSP* triangle_bsp_;

  /** @} */
};

//=============================================================================

#if defined(INCLUDE_TEMPLATES) && !defined(BTBASEMESHOBJECT_C)
#define BTBASEMESHOBJECT_TEMPLATES
#include "BTBaseMeshObjectT_impl.hh"
#endif