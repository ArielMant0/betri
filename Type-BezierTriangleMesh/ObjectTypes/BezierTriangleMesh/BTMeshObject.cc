//== INCLUDES =================================================================

#include <OpenFlipper/common/Types.hh>
#include <ACG/Scenegraph/DrawModes.hh>
#include <OpenFlipper/common/GlobalOptions.hh>
#include <OpenFlipper/BasePlugin/PluginFunctions.hh>

#include <ACG/Scenegraph/ManipulatorNode.hh>
#include <ACG/Scenegraph/BoundingBoxNode.hh>
#include <ACG/QtScenegraph/QtTranslationManipulatorNode.hh>

#include "BTMeshObject.hh"

#ifdef DRAW_CURVED

#define DATA_BEZIER_TRIANGLE_MESH typeId("BezierTriangleMesh")

//== TYPEDEFS =================================================================

//== CLASS DEFINITION =========================================================

// TODO
/** Constructor for BSpline Surface Objects. This object class gets a Separator Node giving
*  the root node to which it should be connected. The spline surface is generated internally
*  and all nodes for visualization will be added below the scenegraph node.\n
*  You dont need to create an object of this type manually. Use
*  PluginFunctions::addBSplineSurface instead. ( see Types.hh::DataType )
*/
/*
BSplineSurfaceObject::BSplineSurfaceObject() :
	BaseObjectData(),
	splineSurface_(NULL),
	splineSurfaceNode_(NULL),
	shaderNode_(0),
	textureNode_(0)

{
	setDataType(DATA_BSPLINE_SURFACE);
	init();
}*/

BTMeshObject::BTMeshObject(const BTMeshObject& _object) :
    BaseObjectData(_object),
    statusNode_(0),
    areaNode_(0),
    handleNode_(0),
    featureNode_(0),
    meshNode_(0),
    textureNode_(0),
    shaderNode_(0),
    statusView_(0),
    triangle_bsp_(0)
{
    init(_object.mesh_);
    setName(name());
}

/** Constructor for Mesh Objects. This object class gets a Separator Node giving
*  the root node to which it should be connected. The mesh is generated internally
*  and all nodes for visualization will be added below the scenegraph node.\n
*  You dont need to create an object of this type manually. Use
*  PluginFunctions::addTriMesh or PluginFunctions::addPolyMesh instead. The
*  objectDataType has to match the one of BezierTMesh ( see Types.hh::DataType )
*/
BTMeshObject::BTMeshObject() :
    BaseObjectData(),
    mesh_(0),
    statusNode_(0),
    areaNode_(0),
    handleNode_(0),
    featureNode_(0),
    meshNode_(0),
    textureNode_(0),
    shaderNode_(0),
    statusView_(0),
    triangle_bsp_(0)
{
    setDataType(DATA_BEZIER_TRIANGLE_MESH);
    init();
}

/** Destructor for Mesh Objects. The destructor deletes the mesh and all
*  Scenegraph nodes associated with the mesh or the object.
*/
BTMeshObject::~BTMeshObject()
{
    // Delete the data attached to this object ( this will remove all perObject data)
    // Not the best way to do it but it will work.
    // This is only necessary if people use references to the mesh below and
    // they do something with the mesh in the destructor of their
    // perObjectData.
    deleteData();

    // Delete the Mesh only, if this object contains a mesh
    if ( mesh_ != NULL)  {
        delete mesh_;
        mesh_ = NULL;
    } else {
        std::cerr << "Destructor error : Mesh already deleted" << std::endl;
    }

    if ( triangle_bsp_ != 0 )
        delete triangle_bsp_;
    triangle_bsp_ = 0;

    // No need to delete the scenegraph Nodes as this will be managed by baseplugin
    areaNode_    = 0;
    handleNode_  = 0;
    featureNode_ = 0;
    meshNode_    = 0;
    textureNode_ = 0;
    shaderNode_  = 0;
    statusView_  = 0;
}

/** Cleanup Function for Mesh Objects. Deletes the contents of the whole object and
* calls BTMeshObject::init afterwards.
*/
void BTMeshObject::cleanup()
{
    // Delete the Mesh only, if this object contains a mesh
    if ( mesh_ != NULL) {
        delete mesh_;
        mesh_ = NULL;
    } else {
        std::cerr << "Cleanup error : Triangle Mesh already deleted" << std::endl;
    }

    if ( triangle_bsp_ != 0 )
        delete triangle_bsp_;
    triangle_bsp_ = 0;

    BaseObjectData::cleanup();

    statusNode_  = 0;
    areaNode_    = 0;
    handleNode_  = 0;
    featureNode_ = 0;
    textureNode_ = 0;
    shaderNode_  = 0;
    meshNode_    = 0;
    statusView_  = 0;

	setDataType(DATA_BEZIER_TRIANGLE_MESH);

    init();
}

//-----------------------------------------------------------------------------

BaseObject* BTMeshObject::copy()
{
	BTMeshObject *object = new BTMeshObject(*this);
	return object;
}

/** This function initalizes the mesh object. It creates the scenegraph nodes,
*  the mesh and requests all required properties for the mesh.
*/
void BTMeshObject::init(BezierTMesh* _mesh)
{
    if (_mesh == 0)
        mesh_ = new BezierTMesh();
    else
        mesh_ = new BezierTMesh(*_mesh);

    // Prepare mesh and request required properties
    mesh_->request_vertex_normals();
    mesh_->request_face_normals();
    mesh_->request_vertex_status();
    mesh_->request_halfedge_status();
    mesh_->request_face_status();
    mesh_->request_edge_status();
    //mesh_->request_vertex_colors();
    //mesh_->request_face_colors();

	// TODO
	// request default properties
	//mesh()->request_controlpoint_selections();
	//mesh()->request_edge_selections();

    // Only initialize scenegraph nodes when we initialized a gui!!
    if (OpenFlipper::Options::nogui())
        return;

    // This should never happen!
    if (manipulatorNode() == NULL)
        std::cerr << "Error when creating Mesh Object! manipulatorNode is NULL!" << std::endl;

	if (materialNode() == NULL)
		std::cerr << "Error when creating BSplineSurface Object! materialNode is NULL!" << std::endl;

    textureNode_ = new ACG::SceneGraph::EnvMapNode(materialNode(),"NEW TextureNode for ", true, GL_LINEAR_MIPMAP_LINEAR );
    shaderNode_  = new ACG::SceneGraph::ShaderNode(textureNode() , "NEW ShaderNode for ");
    meshNode_    = new ACG::SceneGraph::BezierTriangleMeshNode<BezierTMesh>(*mesh_, shaderNode_, "NEW BezierTriangleMeshNode");

    QString shaderDir = OpenFlipper::Options::shaderDirStr() +
		OpenFlipper::Options::dirSeparator();

    std::string shaderDirectory = std::string(shaderDir.toUtf8());
    shaderNode_->setShaderDir(shaderDirectory);

    if(!OpenFlipper::Options::coreProfile())
    {
        if (QFile(shaderDir + "Phong/Vertex.glsl").exists() && QFile( shaderDir + "Phong/Fragment.glsl" ).exists())
            shaderNode_->setShader(ACG::SceneGraph::DrawModes::SOLID_PHONG_SHADED, "Phong/Vertex.glsl", "Phong/Fragment.glsl" );
        else
            std::cerr << "Shader Files for Phong not found!" << std::endl;
    }

    // Node showing selection
    statusNode_ = new ACG::SceneGraph::SelectionNodeT<BezierTMesh>(*mesh_, 0, "NEW StatusNode for mesh " );
    statusNode_->set_point_size(4.0);
    statusNode_->set_color(ACG::Vec4f(1.0f,0.0f,0.0f,1.0f));
    statusNode_->set_base_color(ACG::Vec4f(1.0f,0.0f,0.0f,1.0f));
    // Status nodes are handled specially by their BTStatusViewNodeT parent which
    // is why they get a NONE draw mode.
    statusNode_->drawMode(ACG::SceneGraph::DrawModes::NONE);

    // Node showing modeling region
    areaNode_ = new ACG::SceneGraph::StatusNodeT<BezierTMesh, AreaNodeMod<BezierTMesh> >(*mesh_, 0, "NEW AreaNode for mesh ");
    areaNode_->set_round_points(true);
    areaNode_->enable_alpha_test(0.5);
    areaNode_->set_point_size(7.0);
    areaNode_->set_color(ACG::Vec4f(0.4f, 0.4f, 1.0f, 1.0f));
    // Status nodes are handled specially by their BTStatusViewNodeT parent which
    // is why they get a NONE draw mode.
    areaNode_->drawMode(ACG::SceneGraph::DrawModes::NONE);

    // Node showing handle region
    handleNode_ = new ACG::SceneGraph::StatusNodeT<BezierTMesh, HandleNodeMod<BezierTMesh> >(*mesh_, 0, "NEW HandleNode for mesh ");
    handleNode_->set_round_points(true);
    handleNode_->enable_alpha_test(0.5);
    handleNode_->set_line_width(2.0);
    handleNode_->set_point_size(7.0);
    handleNode_->set_color(ACG::Vec4f(0.2f, 1.0f, 0.2f, 1.0f));
    // Status nodes are handled specially by their BTStatusViewNodeT parent which
    // is why they get a NONE draw mode.
    handleNode_->drawMode(ACG::SceneGraph::DrawModes::NONE);

    // Node showing feature selection
    featureNode_ = new ACG::SceneGraph::StatusNodeT<BezierTMesh, FeatureNodeMod<BezierTMesh> >(*mesh_, 0, "NEW FeatureNode for mesh ");
    featureNode_->set_round_points(true);
    featureNode_->enable_alpha_test(0.5);
    featureNode_->set_line_width(2.0);
    featureNode_->set_point_size(7.0);
    featureNode_->set_color(ACG::Vec4f(1.0f, 0.2f, 1.0f, 1.0f));
    featureNode_->set_base_color(ACG::Vec4f(1.0f, 0.2f, 1.0f, 1.0f));
    // Status nodes are handled specially by their BTStatusViewNodeT parent which
    // is why they get a NONE draw mode.
    featureNode_->drawMode(ACG::SceneGraph::DrawModes::NONE);

	// TODO
    // Link the status nodes to the draw mesh of the mesh below them to reuse its buffers
    //if (meshNode_) {
    //    statusNode_->setDrawMesh(meshNode_->getDrawMesh());
    //    featureNode_->setDrawMesh(meshNode_->getDrawMesh());
    //    areaNode_->setDrawMesh(meshNode_->getDrawMesh());
    //    handleNode_->setDrawMesh(meshNode_->getDrawMesh());
    //}

    // Node rendering selections in correct order
    statusView_ = new ACG::SceneGraph::BTStatusViewNodeT<BezierTMesh>(
        manipulatorNode(),
        "NEW StatusViewNode for mesh ",
        statusNode_,
        areaNode_,
        handleNode_,
        featureNode_
    );

    // make StatusViewNode parent of status nodes
    statusNode_->set_parent(statusView_);
    areaNode_->set_parent(statusView_);
    handleNode_->set_parent(statusView_);
    featureNode_->set_parent(statusView_);

    // Update all nodes
    update();
}

// ===============================================================================
// Name/Path Handling
// ===============================================================================

/** Set the name of an object. All Scenegraph nodes are renamed too. It also calls
* BaseObjectData::setName.
*/
void BTMeshObject::setName(QString _name)
{
    BaseObjectData::setName(_name);

    // No update when gui is not active
    if ( OpenFlipper::Options::nogui() )
        return;

    std::string nodename = std::string("StatusNode for mesh " + _name.toUtf8());
    statusNode_->name( nodename );

    nodename = std::string("AreaNode for mesh " + _name.toUtf8());
    areaNode_->name( nodename );

    nodename = std::string("HandleNode for mesh " + _name.toUtf8());
    handleNode_->name( nodename );

    nodename = std::string("FeatureNode for mesh " + _name.toUtf8());
    featureNode_->name( nodename );

    nodename = std::string("TextureNode for mesh "+ _name.toUtf8());
    textureNode_->name( nodename );

    nodename = std::string("ShaderNode for mesh "+ _name.toUtf8());
    shaderNode_->name( nodename );

    nodename = std::string("BezierTriangleMeshNode for mesh " + _name.toUtf8());
    meshNode_->name( nodename );

    nodename = std::string("StatusViewNode for mesh " + _name.toUtf8());
    statusView_->name( nodename );
}

// ===============================================================================
// Content
// ===============================================================================

/** Get a pointer to the object's mesh.
* @return Pointer to the mesh
*/
BezierTMesh* BTMeshObject::mesh()
{
    return mesh_;
}

/** Get a const pointer to the object's mesh.
* @return Const pointer to the mesh
*/
const BezierTMesh* BTMeshObject::mesh() const
{
    return mesh_;
}

/** Updates the visualization of the object. Calls BTMeshObject::updateGeometry,
*  BTMeshObject::updateTopology, BTMeshObject::updateSelection and
*  BTMeshObject::updateModelingRegions.
*/
void BTMeshObject::update(UpdateType _type)
{
    // No update necessary if no gui
    if (OpenFlipper::Options::nogui())
        return;

    PluginFunctions::setMainGLContext();

    if (_type.contains(UPDATE_ALL) || _type.contains(UPDATE_TOPOLOGY)) {
        updateGeometry();
        updateColor();
        updateTopology();
        updateSelection();
        updateFeatures();
        updateModelingRegions();
        updateTexture();
    } else {
        if (_type.contains(UPDATE_GEOMETRY)) {
            updateGeometry();
        }
        if (_type.contains(UPDATE_SELECTION)) {
            updateSelection();
            updateFeatures();
            updateModelingRegions();
        }
        if (_type.contains(UPDATE_COLOR)) {
            updateColor();
        }
        if (_type.contains(UPDATE_TEXTURE)) {
            updateTexture();
        }
    }
	/*
	if (_type.contains(UPDATE_ALL))
	{
		// mark textures as invalid
		splineSurfaceNode()->cpSelectionTextureValid(false);
		splineSurfaceNode()->knotSelectionTextureValid(false);

		splineSurfaceNode()->updateGeometry();

		if (textureNode())
			splineSurfaceNode()->set_arb_texture(textureNode()->activeTexture());
	}
	else
	{
		if (_type.contains(UPDATE_GEOMETRY) || _type.contains(UPDATE_SELECTION))
		{
			// mark textures as invalid
			splineSurfaceNode()->cpSelectionTextureValid(false);
			splineSurfaceNode()->knotSelectionTextureValid(false);

			if (_type.contains(UPDATE_GEOMETRY))
				splineSurfaceNode()->updateGeometry();
		}
		else if (_type.contains(UPDATE_SELECTION_VERTICES))
			splineSurfaceNode()->cpSelectionTextureValid(false);
		else if (_type.contains(UPDATE_SELECTION_KNOTS))
			splineSurfaceNode()->knotSelectionTextureValid(false);
		if (_type.contains(UPDATE_TEXTURE) && textureNode())
			splineSurfaceNode()->set_arb_texture(textureNode()->activeTexture());

	}*/
}

/** Updates the selection scenegraph nodes */
void BTMeshObject::updateSelection()
{
    if (statusNode_) {
        statusNode_->updateSelection();
    }
}

/** Updates the geometry information in the mesh scenegraph node */
void BTMeshObject::updateGeometry()
{
    if (meshNode_) {
        meshNode_->updateGeometry();
        // Also update the selection nodes here.
        // These nodes store their positions based on the mesh.
        // So they would be at the wrong position afterwards
        statusNode_->updateGeometry();
        featureNode_->updateGeometry();
        areaNode_->updateGeometry();
        handleNode_->updateGeometry();
    }

    invalidateTriangleBsp();
}

/** Updates the color information in the mesh scenegraph node */
void BTMeshObject::updateColor()
{
    //if (meshNode_)
    //    meshNode_->update_color();
}

/** Updates the topology information in the mesh scenegraph node */
void BTMeshObject::updateTopology()
{
    if (meshNode_) {
        //meshNode_->update_topology();
        statusNode_->updateTopology();
        featureNode_->updateTopology();
        areaNode_->updateTopology();
        handleNode_->updateTopology();
    }
    invalidateTriangleBsp();
}

/** Updates the modeling regions scenegraph nodes */
void BTMeshObject::updateModelingRegions()
{
    if ( areaNode_ && handleNode_ ) {
        areaNode_->updateSelection();
        handleNode_->updateSelection();
    }
}

/** Updates the modeling regions scenegraph nodes */
void BTMeshObject::updateFeatures()
{
    if ( featureNode_ )
        featureNode_->updateSelection();
}

/** Updates the modeling regions scenegraph nodes */
void BTMeshObject::updateTexture()
{
    //meshNode_->update_textures();
}


// ===============================================================================
// Visualization
// ===============================================================================
void BTMeshObject::setSelectionColor(const ACG::Vec4f& _color)
{
    if (statusNode_)
    {
        statusNode_->set_color(_color);
        statusNode_->set_base_color(_color);
    }
}

ACG::Vec4f BTMeshObject::selectionColor() const
{
    if (statusNode_)
        return statusNode_->base_color();
    else
        return ACG::Vec4f(-1.f,-1.f,-1.f,-1.f);
}

void BTMeshObject::setAreaColor(const ACG::Vec4f& _color)
{
    if (areaNode_)
    {
        areaNode_->set_color(_color);
        areaNode_->set_base_color(_color);
    }
}

ACG::Vec4f BTMeshObject::areaColor() const
{
    if (areaNode_)
        return areaNode_->base_color();
    else
        return ACG::Vec4f(-1.f,-1.f,-1.f,-1.f);
}

void BTMeshObject::setFeatureColor(const ACG::Vec4f& _color)
{
    if (featureNode_) {
        featureNode_->set_color(_color);
        featureNode_->set_base_color(_color);
    }
}

ACG::Vec4f BTMeshObject::featureColor() const
{
    if (featureNode_)
        return featureNode_->base_color();
    else
        return ACG::Vec4f(-1.f,-1.f,-1.f,-1.f);
}

void BTMeshObject::setHandleColor(const ACG::Vec4f& _color)
{
    if (handleNode_) {
        handleNode_->set_color(_color);
        handleNode_->set_base_color(_color);
    }
}

ACG::Vec4f BTMeshObject::handleColor() const
{
    if (handleNode_)
        return handleNode_->base_color();
    else
        return ACG::Vec4f(-1.f,-1.f,-1.f,-1.f);
}

/** Returns a pointer to the bezierTriangleMesh node
 * @return Pointer to the bezierTriangleMesh node
 */
ACG::SceneGraph::BezierTriangleMeshNode<BezierTMesh>* BTMeshObject::bezierTriangleMeshNode() {
	return meshNode_;
}


/** Returns a pointer to the texture node
* @return Pointer to the texture node
*/
ACG::SceneGraph::EnvMapNode* BTMeshObject::textureNode()
{
    return textureNode_;
}

/** Returns a pointer to the shader node
* @return Pointer to the shader node
*/
ACG::SceneGraph::ShaderNode* BTMeshObject::shaderNode()
{
    return shaderNode_;
}

/** Shows or hides the selections on the object
 */
void BTMeshObject::hideSelection(bool _hide)
{
    if (_hide) {
        statusNode_->set_status( ACG::SceneGraph::BaseNode::HideNode );
    } else {
        statusNode_->set_status( ACG::SceneGraph::BaseNode::Active );
    }
}

void BTMeshObject::hideFeatures(bool _hide)
{
    if (_hide) {
        featureNode_->set_status(ACG::SceneGraph::BaseNode::HideNode);
    } else {
        featureNode_->set_status(ACG::SceneGraph::BaseNode::Active);
    }
}

bool BTMeshObject::featuresVisible() {
    return ( featureNode_->status() == ACG::SceneGraph::BaseNode::Active);
}

/** Shows or hides the areas on the object
 */
void BTMeshObject::hideArea(StatusBits _bit, bool _hide)
{
    ACG::SceneGraph::BaseNode::StatusMode status;

    if (_hide)
        status = ACG::SceneGraph::BaseNode::HideNode;
    else
        status = ACG::SceneGraph::BaseNode::Active;

    if (_bit & AREA)
        areaNode_->set_status(status);
    if (_bit & HANDLEAREA)
        handleNode_->set_status(status);
}

bool BTMeshObject::selectionVisible()
{
    return ( statusNode_->status() == ACG::SceneGraph::BaseNode::Active );
}

bool BTMeshObject::areaVisible( StatusBits _bit )
{
    bool status = true;

    if ( _bit & AREA )
        status &= ( areaNode_->status()   == ACG::SceneGraph::BaseNode::Active );

    if ( _bit & HANDLEAREA )
        status &= ( handleNode_->status() == ACG::SceneGraph::BaseNode::Active );

    return status;
}

/** Get the Bounding box size of this object
 */
void BTMeshObject::boundingBox( ACG::Vec3d& _bbMin , ACG::Vec3d& _bbMax )
{
    if ( meshNode_ ) {
        _bbMin = ACG::Vec3d(FLT_MAX, FLT_MAX, FLT_MAX);
        _bbMax = ACG::Vec3d(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        meshNode_->boundingBox(_bbMin,_bbMax);
    } else {
        std::cerr << "Error: Bounding box computation via Scenegraph not available without gui" << std::endl;
    }
}

// ===============================================================================
// Load/Save
// ===============================================================================

/** Load the mesh data from a file. The loader uses the OpenMesh object loaders and
* therefore supports all mesh types supported by OpenMesh.
*
* @param _filename Full path of the file to load.
*/
bool BTMeshObject::loadMesh(QString _filename)
{
    setFromFileName(_filename);

    // call the local function to update names
    setName(name());

    std::string filename = std::string( _filename.toUtf8());

    // load file
    bool ok = OpenMesh::IO::read_mesh( (*mesh()) , filename);
    if (!ok)
    {
        if (dataType() == typeId("TriangleMesh")) // TODO
        std::cerr << "Main Application : Read error for Triangle Mesh at "<< filename << std::endl;
        if (dataType() == typeId("PolyMesh"))
        std::cerr << "Main Application : Read error for Poly Mesh\n";
        return false;
    }

    mesh()->update_normals();

    update();

    show();

    return true;
}

// ===============================================================================
// Object information
// ===============================================================================

/** Returns a string containing all information about the current object. This also
* includes the information provided by BaseObjectData::getObjectinfo
*
* @return String containing the object information
*/
QString BTMeshObject::getObjectinfo()
{
    QString output;

    output += "========================================================================\n";
    output += BaseObjectData::getObjectinfo();

    if (dataType(typeId("TriangleMesh"))) // TODO
        output += "Object Contains Triangle Mesh : ";

    if (dataType(typeId("PolyMesh")))
        output += "Object Contains Poly Mesh : ";

    output += QString::number( mesh()->n_vertices() ) + " vertices, ";
    output += QString::number( mesh()->n_edges() ) += " edges ";
    output += QString::number( mesh()->n_faces() ) += " faces.\n";

	// output += QString::number(splineSurface()->n_control_points_m()) + " control points in m direction, ";
	// output += QString::number(splineSurface()->n_control_points_n()) + " control points in n direction, ";

    output += "========================================================================\n";
    return output;
}

// ===============================================================================
// Picking
// ===============================================================================

/** Given an node index from PluginFunctions::scenegraphPick this function can be used to
* check if the meshNode of the object has been picked.
*
* @param _node_idx Index of the picked mesh node
* @return bool if the meshNode of this object is the picking target.
*/
bool BTMeshObject::picked(uint _node_idx)
{
    return (_node_idx == meshNode_->id());
}

void BTMeshObject::enablePicking(bool _enable)
{
    if (OpenFlipper::Options::nogui())
        return;

    meshNode_->enablePicking( _enable );
    areaNode_->enablePicking( _enable );
    handleNode_->enablePicking( _enable );
    featureNode_->enablePicking( _enable );
    textureNode_->enablePicking( _enable );
    shaderNode_->enablePicking( _enable );
}

bool BTMeshObject::pickingEnabled()
{
    return meshNode_->pickingEnabled();
}

/// Refine picking on triangle meshes
ACG::Vec3d BTMeshObject::refinePick(
	ACG::SceneGraph::PickTarget _pickTarget,
	const ACG::Vec3d _hitPoint,
	const ACG::Vec3d _start,
	const ACG::Vec3d _dir,
	const unsigned int _targetIdx
) {
	if (_pickTarget == ACG::SceneGraph::PICK_FACE) {
		// get picked face handle
		BezierTMesh::FaceHandle fh = mesh()->face_handle(_targetIdx);

		BezierTMesh::FaceVertexIter fv_it = mesh()->fv_begin(fh);

		// Get vertices of the face
		ACG::Vec3d p1 = mesh()->point(*fv_it);
		++fv_it;

		ACG::Vec3d p2 = mesh()->point(*fv_it);
		++fv_it;

		ACG::Vec3d p3 = mesh()->point(*fv_it);
		++fv_it;

		ACG::Vec3d hitpointNew = _hitPoint;

		BezierTMesh::Scalar t, u, v;
		if (ACG::Geometry::triangleIntersection(_start, _dir, p1, p2, p3, t, u, v)) {
			hitpointNew = _start + t * _dir;
		}

		return hitpointNew;
	}

	if (_pickTarget == ACG::SceneGraph::PICK_EDGE) {
		// get picked edge handle
		BezierTMesh::EdgeHandle eh = mesh()->edge_handle(_targetIdx);
		if (eh.is_valid()) {
			BezierTMesh::HalfedgeHandle heh = mesh()->halfedge_handle(eh, 0);

			//get vertices of the edge
			BezierTMesh::VertexHandle vhbegin = mesh()->to_vertex_handle(heh);
			BezierTMesh::VertexHandle vhend = mesh()->from_vertex_handle(heh);
			ACG::Vec3d edgeStart = mesh()->point(vhbegin);
			ACG::Vec3d edgeEnd = mesh()->point(vhend);

			//retrieve the point on the edge that is closest to the backprojected hitpoint
			ACG::Vec3d hitPointNew;
			ACG::Geometry::distPointLineSquared(_hitPoint, edgeStart, edgeEnd, &hitPointNew);

			return hitPointNew;
		}
	}

	if (_pickTarget == ACG::SceneGraph::PICK_VERTEX) {
		// get picked vertex handle
		BezierTMesh::VertexHandle vh = mesh()->vertex_handle(_targetIdx);
		if (vh.is_valid()) {
			ACG::Vec3d hitpointNew = mesh()->point(vh);
			//just return the vertex position
			return hitpointNew;
		}
	}

	return _hitPoint;
}

//=============================================================================


// ===============================================================================
// Octree
// ===============================================================================

typename BTMeshObject::OMTriangleBSP* BTMeshObject::requestTriangleBsp()
{
    if (!dataType(typeId("TriangleMesh"))) {
        std::cerr << "Bsps are only supported for triangle meshes." << std::endl;
        return 0;
    }

    // Create the tree if needed.
    if ( triangle_bsp_ == 0 ) {
        // create Triangle BSP
        triangle_bsp_ = new OMTriangleBSP( *mesh() );

        // build Triangle BSP
        triangle_bsp_->reserve(mesh()->n_faces());

        typename BezierTMesh::FIter f_it  = mesh()->faces_begin();
        typename BezierTMesh::FIter f_end = mesh()->faces_end();

        for (; f_it!=f_end; ++f_it)
        triangle_bsp_->push_back(*f_it);

        triangle_bsp_->build(10, 100); //max vertices per leaf 10, max depth 100
    }

    // return pointer to triangle bsp
    return triangle_bsp_;
}

typename BTMeshObject::OMTriangleBSP* BTMeshObject::resetTriangleBsp()
{
    if (triangle_bsp_ != 0) {
        delete triangle_bsp_;
        triangle_bsp_ = 0;
    }

    return requestTriangleBsp();
}

void BTMeshObject::invalidateTriangleBsp()
{
    if ( triangle_bsp_ != 0 ) {
        delete triangle_bsp_;
        triangle_bsp_ = 0;
    }
}

bool BTMeshObject::hasBsp() const
{
    return triangle_bsp_ != 0;
}

BaseNode* BTMeshObject::primaryNode()
{
    return boundingBoxNode();
}

#endif