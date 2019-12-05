///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////
#include "BezierTMesh.hh"

#include <OpenFlipper/libs_required/ACG/GL/DrawMesh.hh>

#include <ACG/GL/globjects.hh>
#include <ACG/GL/VertexDeclaration.hh>
#include <ACG/GL/GLPrimitives.hh>

///////////////////////////////////////////////////////////////////////////////
// Namespaces
///////////////////////////////////////////////////////////////////////////////
namespace ACG
{

///////////////////////////////////////////////////////////////////////////////
// CLASS DEFINITION
///////////////////////////////////////////////////////////////////////////////
class DrawBTMesh : public DrawMeshT<BezierTMesh>
{
	// typedefs for easy access
	typedef typename BezierTMesh::Point Point;

private:
	enum REBUILD_TYPE
	{
		REBUILD_NONE = 0, REBUILD_FULL = 1, REBUILD_GEOMETRY = 2, REBUILD_TOPOLOGY = 4, REBUILD_TEXTURES = 8
	};

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////
public:
	explicit DrawBTMesh(BezierTMesh& _mesh) :
		DrawMeshT<BezierTMesh>(_mesh),
		bezierTriangleMesh_(_mesh),
		rebuild_(REBUILD_FULL), // TODO before REBUILD_NONE

		prevNumFaces_(0), prevNumVerts_(0),
		invVertexMap_(0),
		// Mode settings
		colorMode_(1), curVBOColorMode_(1),
		flatMode_(0), bVBOinFlatMode_(0),
		textureMode_(1), bVBOinHalfedgeTexMode_(1),
		halfedgeNormalMode_(0), bVBOinHalfedgeNormalMode_(0),
		// Offsets
		offsetPos_(0), offsetNormal_(20), offsetTexc_(12), offsetColor_(32),

		textureIndexPropertyName_("Not Set"),

		invalidateSurfaceMesh_(true),
		surfaceIndexCount_(0),
		NEWVERTICES(0), // TODO
		VERTEXSUM(3), // TODO
		STEPSIZE(1.0) // TODO
	{
	}

	///////////////////////////////////////////////////////////////////////////
	// Add Object
	///////////////////////////////////////////////////////////////////////////
	void addTriRenderObjects(
		IRenderer* _renderer, const RenderObject* _baseObj,
		std::map<int, GLuint>* _textureMap, bool _nonindexed
	);

	void addPatchRenderObjects(
		IRenderer* _renderer, const RenderObject* _baseObj,
		std::map<int, GLuint>* _textureMap, bool _nonindexed
	);

	void bindBuffersToRenderObject(RenderObject* _baseObj);

	void updateGPUBuffers();

	void updateSurfaceMesh(const int meshOption);

	void rebuild();

	//========================================================================
	// Setup vertex Declaration
	//========================================================================

	void createVertexDeclaration();
	const void* getMeshPropertyType(OpenMesh::BaseProperty* _prop, GLuint* _outType, unsigned int* _outSize) const;
	template<class T>
	const void* testMeshPropertyTypeT(const OpenMesh::BaseProperty* _prop, unsigned int* _outSize) const;

	//========================================================================
	// Setup VBO / IBO
	//========================================================================

	void createVBO();
	void createIBO();

	//========================================================================
	// Utility
	//========================================================================

	unsigned int countTris(unsigned int* pMaxVertsOut, unsigned int* _pOutNumIndices);

	BezierTMesh::HalfedgeHandle mapToHalfedgeHandle(size_t _vertexId);

	void readVertex(
		size_t _vertex,
		const BezierTMesh::VertexHandle&   _vh,
		const BezierTMesh::HalfedgeHandle& _hh,
		const BezierTMesh::FaceHandle&     _fh
	);

	void writeVertexElement(
		void* _dstBuf,
		size_t _vertex, size_t _stride,
		size_t _elementOffset, size_t _elementSize,
		const void* _elementData
	);

	void writePosition(size_t _vertex, const ACG::Vec3d& _n);
	void writeNormal(size_t _vertex, const ACG::Vec3d& _n);
	void writeTexcoord(size_t _vertex, const ACG::Vec2f& _uv);
	void writeColor(size_t _vertex, unsigned int _color);

	unsigned int getVertexColor(const BezierTMesh::VertexHandle& _vh);
	unsigned int getFaceColor(const BezierTMesh::FaceHandle& _fh);

	//========================================================================
	// texture handling
	//========================================================================

	int getTextureIDofFace(unsigned int _face);

	///////////////////////////////////////////////////////////////////////////
	// CPU Tesselation
	///////////////////////////////////////////////////////////////////////////
private:
	int pointsBefore(int level);

	BezierTMesh::Point getCP(
		int i, int j, int k, BezierTMesh::FaceHandle fh
	);

	BezierTMesh::Point oneEntry(
		int i, int j, int k,
		BezierTMesh::Point baryCoords, BezierTMesh::FaceHandle fh
	);

	BezierTMesh::Point newPosition(
		BezierTMesh::Point baryCoords, BezierTMesh::FaceHandle fh
	);

	void VBOtesselatedFromMesh();

	///////////////////////////////////////////////////////////////////////////
	// GPU Tesselation / Normal
	///////////////////////////////////////////////////////////////////////////
	void VBOfromMesh();

	///////////////////////////////////////////////////////////////////////////
	// BoundingVolumes for Raytracing
	///////////////////////////////////////////////////////////////////////////
	void VBOfromBoundingMesh();

///////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////
private:
	BezierTMesh& bezierTriangleMesh_;

	// surface mesh
	GeometryBuffer surfaceVBO_;
	IndexBuffer surfaceIBO_;
	VertexDeclaration surfaceDecl_;
	int surfaceIndexCount_;


	/**
	 * inverse vertex map: original OpenMesh vertex index -> one vertex index in vbo
	 * this map is ambiguous and only useful for per vertex attributes rendering i.e. lines!
	 */
	unsigned int* invVertexMap_;

	//========================================================================
	// Keep track on rebuilds
	//========================================================================

	/// hint on what to rebuild
	unsigned int rebuild_;

	/** used to track mesh changes, that require a full rebuild
	 * values directly taken from Mesh template
	 */
	size_t prevNumFaces_;
	size_t prevNumVerts_;

	//========================================================================
	// Mode settings
	//========================================================================

	/// Color Mode: 0: none, 1: per vertex,  else: per face
	int colorMode_;

	/// Color Mode of vbo
	int curVBOColorMode_;

	/// flat / smooth shade mode toggle
	int flatMode_;

	/// normals in VBO currently in flat / smooth mode
	int bVBOinFlatMode_;

	/// per vertex / halfedge texture mode toggle:  0: per vertex,  1: per halfedge
	int textureMode_;

	/// texcoords in VBO currently in per vertex / halfedge mode toggle
	int bVBOinHalfedgeTexMode_;

	/// per vertex / halfedge normals mode toggle:  0: per vertex,  1: per halfedge
	int halfedgeNormalMode_;

	/// normals in VBO currently in per vertex / halfedge mode toggle
	int bVBOinHalfedgeNormalMode_;

	//========================================================================
	// texture handling
	//========================================================================

	/** \brief Property for the per face texture index.
	 *
	 * This property is used by the mesh for texture index specification.
	 * If this is invalid, then it is assumed that there is one or no active
	 * texture. This means that the generated strips will be independent of texture
	 * information.
	 */
	std::string textureIndexPropertyName_;

	//========================================================================
	// flexible vertex layout
	//========================================================================

	struct VertexProperty
	{
		// get property from vertex, face or halfedge array
		PropertySource source_;

		/// property name in openmesh
		std::string name_;

		/// input name id in vertex shader
		std::string vertexShaderInputName_;

		/// property type as stored in openmesh
		VertexElement sourceType_;

		/// property type as stored in vbo
		VertexElement destType_;

		/// memory address of property data
		const void* propDataPtr_;

		/// element id in vertex declaration
		int declElementID_;
	};

	/// fixed vertex elements:
	const size_t offsetPos_;
	const size_t offsetNormal_;
	const size_t offsetTexc_;
	const size_t offsetColor_;

	/// additional optional elements
	std::vector<VertexProperty> additionalElements_;

///////////////////////////////////////////////////////////////////////////////
// Tessellation variables
///////////////////////////////////////////////////////////////////////////////
public:
	bool invalidateSurfaceMesh_; // TODO

	// Additional Vertices per edge trough subdivision
	int NEWVERTICES;

	// Sum of all new Trianglevertices
	int VERTEXSUM;

	// TODO does it make sence to have two different stepsizes?
	// TODO Capital letters?
	// TODO calculate the other two values if one of them is given
	double STEPSIZE;
};

//=============================================================================
//} // namespace betri
} // namespace ACG
//=============================================================================