#pragma once
/**
 * \file BezierTriangleMeshNode.hh
 *
 */

//== INCLUDES =================================================================

#include <ACG/Scenegraph/BaseNode.hh>
#include <ACG/Scenegraph/DrawModes.hh>

#include <QGLWidget>

#include <QImage>

#include <OpenFlipper/BasePlugin/PluginFunctions.hh> // TODO

#include <ACG/GL/globjects.hh>
#include <ACG/GL/VertexDeclaration.hh>
#include <ACG/GL/GLPrimitives.hh>

#include "BezierTMesh.hh"

//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

namespace ACG {
namespace SceneGraph {

//== CLASS DEFINITION =========================================================


/** \class BezierTriangleMeshNode BezierTriangleMeshNode.hh <ACG/.../BezierTriangleMeshNode.hh>
  Brief Description.

  A more elaborate description follows.
*/

template <class MeshT>
class BezierTriangleMeshNode : public BaseNode
{
public:

	// typedefs for easy access
	typedef typename MeshT::Point Point;

	/// Constructor
	BezierTriangleMeshNode(MeshT& _bss,
		BaseNode*    _parent = 0,
		std::string  _name = "<BezierTriangleMeshNode>") :
		BaseNode(_parent, _name),
		bezierTriangleMesh_(_bss),
		bspline_draw_mode_(NORMAL),
		bspline_selection_draw_mode_(NONE),
		pick_radius_(1.0),
		resolution_(16),
		controlnet_color_(Vec4f(34.f / 255.f, 139.f / 255.f, 34.f / 255.f, 1.f)),
		controlpoints_color_(Vec4f(255.f / 255.f, 0.f / 255.f, 0.f / 255.f, 1.f)),
		controlnet_highlight_color_(Vec4f(1.0f, 1.0f, 1.0f, 1.0f)),
		surface_color_(Vec4f(178.0f / 255.0f, 34.0f / 255.0f, 34.0f / 255.0f, 1.0f)),
		surface_highlight_color_(Vec4f(1.0f, 1.0f, 1.0f, 1.0f)),
		render_control_net_(false), // TODO
		render_bspline_surface_(true),
		adaptive_sampling_(false),
		controlPointSelectionTexture_valid_(false),
		knotVectorSelectionTexture_valid_(false),
		pick_texture_idx_(0),
		pick_texture_res_(256),
		pick_texture_baseidx_(0),
		cp_selection_texture_idx_(0),
		cp_selection_texture_res_(256),
		knot_selection_texture_idx_(0),
		knot_selection_texture_res_(256),
		arb_texture_idx_(0),
		arb_texture_used_(false),
		arb_texture_repeat_(false),
		arb_texture_repeat_u_(1.0),
		arb_texture_repeat_v_(1.0),
		surfaceIndexCount_(0),
		invalidateSurfaceMesh_(true),
		controlNetSelIndices_(0),
		controlNetLineIndices_(0),
		invalidateControlNetMesh_(true),
		invalidateControlNetMeshSel_(true),
		controlPointsChangedR_(true), // TODO
		controlPointsChangedC_(true), // TODO
		oldFaceCount_(0), // TODO
		NEWVERTICES(0), // TODO
		VERTEXSUM(3), // TODO
		STEPSIZE(1.0) // TODO
	{
		cylinder_ = new GLCylinder(16, 1, 1.0f, true, true);
		sphere_ = new GLSphere(5, 5);
		fancySphere_ = new GLSphere(16, 16);
		PluginFunctions::setDrawMode(ACG::SceneGraph::DrawModes::SOLID_FACES_COLORED);
	}

	/// Destructor
	~BezierTriangleMeshNode()
	{
		delete cylinder_;
		delete sphere_;
		delete fancySphere_;
	}

	enum BTDrawMode {
		NORMAL = 0,
		FANCY = 1
	};

	enum BTSelectionDrawMode {
		NONE = 0,
		CONTROLPOINT = 1,
		KNOTVECTOR = 2
	};


	MeshT& bsplineSurface() {
		return bezierTriangleMesh_;
	}

	void set_pick_radius(double _pr) {
		pick_radius_ = _pr;
	}

	/// static name of this class
	ACG_CLASSNAME(BezierTriangleMeshNode);

	/// return available draw modes
	DrawModes::DrawMode availableDrawModes() const;

	/// update bounding box
	void boundingBox(Vec3d& _bbMin, Vec3d& _bbMax);

	// TODO
	void setControlPointsCircular();
	void setControlPointsColumnwise();

	/// draw lines and normals
	void draw(GLState& _state, const DrawModes::DrawMode& _drawMode);

	/// create render objects
	void getRenderObjects(IRenderer* _renderer, GLState& _state, const DrawModes::DrawMode& _drawMode, const Material* _mat);

	/// picking
	void pick(GLState& _state, PickTarget _target);

	/// update vertex buffer for rendering
	void updateGeometry();

	void set_rendering_resolution(int _res) {
		resolution_ = _res;
	};

	void render_control_net(bool _render) {
		render_control_net_ = _render;
	};

	bool render_control_net() {
		return render_control_net_;
	};

	void render_bspline_surface(bool _render) {
		render_bspline_surface_ = _render;
	};

	bool render_bspline_surface() {
		return render_bspline_surface_;
	};

	void set_bspline_draw_mode(BTDrawMode _mode) {
		bspline_draw_mode_ = _mode;
	};

	void set_selection_draw_mode(BTSelectionDrawMode _mode) {
		bspline_selection_draw_mode_ = _mode;
	};

	BTSelectionDrawMode get_selection_draw_mode() const {
		return bspline_selection_draw_mode_;
	}

	void adaptive_sampling(bool _adaptive) {
		adaptive_sampling_ = _adaptive;
	};

	void cpSelectionTextureValid(bool _valid) {
		controlPointSelectionTexture_valid_ = _valid;
	};
	void knotSelectionTextureValid(bool _valid) {
		knotVectorSelectionTexture_valid_ = _valid;
	};


	//! Should be a power of 2
	int& pick_texture_res() {
		return pick_texture_res_;
	}

	/// use arbitrary texture (in SOLID_TEXTURED mode)
	void set_arb_texture(const QImage& _texture, bool _repeat = false, float _u_repeat = 1.0f, float _v_repeat = 1.0f);
	void set_arb_texture(const GLuint _texture) {
		arb_texture_idx_ = _texture;
	}
	void set_repeat_arb_texture(bool _repeat = true, float _urep = 5.0f, float _vrep = 5.0f) {
		arb_texture_repeat_ = _repeat; arb_texture_repeat_u_ = _urep; arb_texture_repeat_v_ = _vrep;
	}
	bool get_repeat_arb_texture() {
		return arb_texture_repeat_;
	}

private:

	void pick_vertices(GLState& _state);
	void pick_spline(GLState& _state);
	void pick_surface(GLState& _state, unsigned int _offset);

	void draw_cylinder(const Point& _p0, const Point& _axis, double _r, GLState& _state);
	void draw_sphere(const Point& _p0, double _r, GLState& _state, GLSphere* _sphere);

	/// Copy constructor (not used)
	BezierTriangleMeshNode(const BezierTriangleMeshNode& _rhs);

	/// Assignment operator (not used)
	BezierTriangleMeshNode& operator=(const BezierTriangleMeshNode& _rhs);

	void render(GLState& _state, bool _fill);

	void drawSurface(GLState& _state, bool _fill = true);

	void drawTexturedSurface(GLState& _state, GLuint _texture_idx);

	void drawControlNet(GLState& _state);

	void drawFancyControlNet(GLState& _state);

	void updateControlPointSelectionTexture(GLState& _state);

	void updateKnotVectorSelectionTexture(GLState& _state);

	/** spline surface u,v-parameter picking */
	/// generate index and setup texture parameters
	void pick_init_texturing();
	/// create texture image
	void pick_create_texture(GLState& _state);

	/// draw textured nurbs patch
	void pick_draw_textured_nurbs(GLState& _state);

	/// generate index and setup texture parameters for selection visualization
	void selection_init_texturing(GLuint & _texture_idx);

	/// creates texture to put onto nurbs curve for visualization of control point selection
	void create_cp_selection_texture(GLState& _state);
	/// creates texture to put onto nurbs curve for visualization of knotvector selection
	void create_knot_selection_texture(GLState& _state);

	/// draw textured nurbs patch
	void draw_textured_nurbs(GLState& _state);

	ACG::Vec4f generateHighlightColor(ACG::Vec4f _color);

	BezierTMesh::Point evaluateCasteljau(Point at, Point cp0, Point cp1, Point cp2, Point cp3, Point cp4, Point cp5);
	void tesselateMeshCPU();  // TODO richtige stelle finden

	/// update vertex + index buffer of surface mesh
	void updateSurfaceMesh(const int meshOption);//int _vertexCountU = 50, int _vertexCountV = 50); TODO

	void VBOfromMesh();

	// TODO ordering
	void VBOfromBoundingMesh();

	int pointsBefore(int level);
	BezierTMesh::Point getCP(int i, int j, int k, BezierTMesh::FaceHandle fh);
	BezierTMesh::Point oneEntry(int i, int j, int k, BezierTMesh::Point baryCoords, BezierTMesh::FaceHandle fh);
	BezierTMesh::Point newPosition(BezierTMesh::Point baryCoords, BezierTMesh::FaceHandle fh);
	void VBOtesselatedFromMesh();

	/// update vertex + index buffer of control net mesh
	void updateControlNetMesh();

	/// update index buffer of selected control points
	void updateControlNetMeshSel();

	/// update texture resources for gpu-based spline evaluation
	void updateTexBuffers();

///////////////////////////////////////////////////////////////////////////////
// Private Membervariables
///////////////////////////////////////////////////////////////////////////////

private:

	MeshT& bezierTriangleMesh_;

	BTDrawMode bspline_draw_mode_;
	BTSelectionDrawMode bspline_selection_draw_mode_;

	double pick_radius_;

	int resolution_;

	Vec4f controlnet_color_;
	Vec4f controlpoints_color_; // TODO
	Vec4f controlnet_highlight_color_;

	Vec4f surface_color_;
	Vec4f surface_highlight_color_;

	bool render_control_net_;
	bool render_bspline_surface_;

	bool adaptive_sampling_;

	bool controlPointSelectionTexture_valid_;
	bool knotVectorSelectionTexture_valid_;

	QImage pick_texture_image_;
	GLuint pick_texture_idx_;
	int    pick_texture_res_;
	// used to only re-create pick_texture_image_ if picking indices changed...
	unsigned int pick_texture_baseidx_;

	// texturing stuff for control point selection highlighting
	QImage cp_selection_texture_image_;
	GLuint cp_selection_texture_idx_;
	int    cp_selection_texture_res_;

	// texturing stuff for knot vector selection highlighting
	QImage knot_selection_texture_image_;
	GLuint knot_selection_texture_idx_;
	int    knot_selection_texture_res_;

	// texturing stuff for using arbitrary textures
	QImage arb_texture_image_;
	GLuint arb_texture_idx_;
	bool   arb_texture_used_;
	bool   arb_texture_repeat_;
	float  arb_texture_repeat_u_;
	float  arb_texture_repeat_v_;

	GLCylinder* cylinder_;
	GLSphere* sphere_;
	GLSphere* fancySphere_;


	// surface mesh
	GeometryBuffer surfaceVBO_;
	IndexBuffer surfaceIBO_;
	VertexDeclaration surfaceDecl_;
	int surfaceIndexCount_;
	bool invalidateSurfaceMesh_;

	// control net mesh
	GeometryBuffer controlNetVBO_;
	IndexBuffer controlNetSelIBO_;
	int controlNetSelIndices_;
	IndexBuffer controlNetLineIBO_;
	int controlNetLineIndices_;
	VertexDeclaration controlNetDecl_;
	bool invalidateControlNetMesh_;
	bool invalidateControlNetMeshSel_;

	// TODO
	bool controlPointsChangedC_;
	bool controlPointsChangedR_;
	int oldFaceCount_;

	// GPU based evaluation
	TextureBuffer knotTexBufferU_;
	TextureBuffer knotTexBufferV_;
	Texture2D controlPointTex_;


///////////////////////////////////////////////////////////////////////////////
// Tessellation variables
///////////////////////////////////////////////////////////////////////////////

	// Additional Vertices per edge trough subdivision
	int NEWVERTICES;
	//static int NEWVERTICES = betri::mersennePrime(ITERATIONS);

	// Sum of all new Trianglevertices
	//static int VERTEXSUM = betri::gaussSum(NEWVERTICES + 2);
	int VERTEXSUM;
	// TODO does it make sence to have two different stepsizes?
	// TODO Capital letters?
	// TODO calculate the other two values if one of them is given
	//static double STEPSIZE = 1.0 / (double(NEWVERTICES) + 1.0);
	double STEPSIZE;

};

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
#if defined(INCLUDE_TEMPLATES) && !defined(ACG_BEZIERTRIANGLEMESHNODE_C)
#define ACG_BEZIERTRIANGLEMESHNODE_TEMPLATES
#include "BezierTriangleMeshNode_impl.hh"
#endif

