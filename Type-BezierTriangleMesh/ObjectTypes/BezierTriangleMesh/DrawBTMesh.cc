///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////
#include "DrawBTMesh.hh"

#include <OpenFlipper/BasePlugin/PluginFunctions.hh> // TODO

#include "globals/BezierOptions.hh"
#include "boundVol/BVolGenerator.hh"

///////////////////////////////////////////////////////////////////////////////
// Defines
///////////////////////////////////////////////////////////////////////////////

#define ITERATIONS betri::option(betri::BezierOption::TESSELLATION_AMOUNT)

///////////////////////////////////////////////////////////////////////////////
// Namespaces
///////////////////////////////////////////////////////////////////////////////
namespace ACG
{
//namespace betri
//{

///////////////////////////////////////////////////////////////////////////////
// Constants
///////////////////////////////////////////////////////////////////////////////
static const int GRAD = 2; // 1 = linear, 2 = quadratisch
//static const int ITERATIONS = 0;

// Additional Control Points per edge
static const int CPCOUNT = GRAD - 1;
// Sum of all Control Polygon Vertices per Face
static const int CPSUM = betri::gaussSum(CPCOUNT + 2);

static const int controlPointsPerFace = CPSUM;

///////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////

void DrawBTMesh::addTriRenderObjects(
	IRenderer* _renderer, const RenderObject* _baseObj,
	std::map<int, GLuint>* _textureMap, bool _nonindexed
)
{
	if (numTris_ || numTris_ != bezierTriangleMesh_.n_faces()) {
		RenderObject ro = *_baseObj;
		if (!_nonindexed) {
			bindBuffersToRenderObject(&ro);
		} else {
			// TODO
			std::cerr << "In addTriRenderObjects: This should not happen! " << std::endl;

			updateFullVBO();

			//ro.vertexBuffer = vboFull_.id();
			ro.vertexDecl = vertexDecl_;
		}

		// textured mode
		if (_baseObj->shaderDesc.textured()) {

			// TODO
			std::cerr << "In addTriRenderObjects: This should not happen! " << std::endl;

			for (int i = 0; i < meshComp_->getNumSubsets(); ++i) {
				const MeshCompiler::Subset* sub = meshComp_->getSubset(i);

				if (_textureMap) {
					if (_textureMap->find(sub->id) == _textureMap->end()) {
						std::cerr << "Illegal texture index ... trying to access " << sub->id << std::endl;
					} else {
						RenderObject::Texture tex;
						tex.type = GL_TEXTURE_2D;
						tex.id = (*_textureMap)[sub->id];
						ro.addTexture(tex, 0);
					}
				} else // no texture map specified, use whatever texture is currently bound to the first texture stage
				{
					GLState::activeTexture(GL_TEXTURE0);
					GLint textureID = 0;
					glGetIntegerv(GL_TEXTURE_BINDING_2D, &textureID);

					RenderObject::Texture tex;
					tex.type = GL_TEXTURE_2D;
					tex.id = textureID;
					ro.addTexture(tex, 0);
				}

				if (!_nonindexed)
					ro.glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(sub->numTris * 3), indexType_,
					(GLvoid*)((size_t)sub->startIndex * (indexType_ == GL_UNSIGNED_INT ? 4 : 2))); // offset in bytes
				else
					ro.glDrawArrays(GL_TRIANGLES, sub->startIndex, static_cast<GLsizei>(sub->numTris * 3));

				_renderer->addRenderObject(&ro);
			}
		} else {
			if (!_nonindexed)
				ro.glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(numTris_ * 3), indexType_, 0);
			else
				ro.glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(numTris_ * 3));
			_renderer->addRenderObject(&ro);
		}
	}
}

void DrawBTMesh::addPatchRenderObjects(
	IRenderer* _renderer, const RenderObject* _baseObj,
	std::map<int, GLuint>* _textureMap, bool _nonindexed
)
{
	if (numTris_ || numTris_ != bezierTriangleMesh_.n_faces()) {
		RenderObject ro = *_baseObj;
		if (!_nonindexed) {
			bindBuffersToRenderObject(&ro);
		} else {
			// TODO
			std::cerr << "In addPatchRenderObjects: This should not happen! " << std::endl;

			updateFullVBO();

			//ro.vertexBuffer = vboFull_.id();
			ro.vertexDecl = vertexDecl_;
		}

		// textured mode
		if (_baseObj->shaderDesc.textured()) {

			// TODO a lot of stuff deleted
			std::cerr << "In addTriRenderObjects: This should not happen! " << std::endl;

		} else {
			ro.glDrawElements(GL_PATCHES, static_cast<GLsizei>(numTris_ * 3), GL_UNSIGNED_INT, 0);

			_renderer->addRenderObject(&ro);
		}
	}
}

void DrawBTMesh::bindBuffersToRenderObject(RenderObject* _obj)
{
	int renderOption = betri::option(betri::BezierOption::TESSELLATION_TYPE);
	int showBVolume = betri::option(betri::BezierOption::SHOW_BOUNDING_VOLUME);

	updateSurfaceMesh(renderOption);
	//updateGPUBuffers();

	//_obj->vertexBuffer = vbo_;
	//_obj->indexBuffer = ibo_;

	_obj->indexType = indexType_;

	// assign correct vertex declaration
	//_obj->vertexDecl = vertexDecl_;

	_obj->vertexBuffer = surfaceVBO_.id();
	_obj->indexBuffer = surfaceIBO_.id();
	_obj->vertexDecl = &surfaceDecl_;
}

void DrawBTMesh::updateGPUBuffers()
{
	// rebuild if necessary
	if ((!numTris_ && bezierTriangleMesh_.n_faces()) ||
		!numVerts_ ||
		(!meshComp_ && bezierTriangleMesh_.n_faces())
	) {
		rebuild_ = REBUILD_FULL;
	}

	//rebuild_ = REBUILD_FULL;
	// TODO
	//if (bVBOinHalfedgeNormalMode_ != halfedgeNormalMode_)
	//	rebuild_ = REBUILD_FULL;

	// if no rebuild necessary, check for smooth / flat shading switch
	// to update normals
	if (rebuild_ == REBUILD_NONE) {
		/*
		if (bVBOinFlatMode_ != flatMode_ ||
			bVBOinHalfedgeTexMode_ != textureMode_ ||
			(colorMode_ && curVBOColorMode_ != colorMode_)
		) {
			createVBO();
		}
		*/
	} else {
		rebuild();
	}
}

void DrawBTMesh::rebuild()
{
	if (rebuild_ == REBUILD_NONE)
		return;

	if (!bezierTriangleMesh_.n_vertices()) {
		numVerts_ = 0;
		numTris_ = 0;
		return;
	}

	// update layout declaration
	createVertexDeclaration();

	/* TODO
	// support for point clouds:
	if (bezierTriangleMesh_.n_vertices() && bezierTriangleMesh_.n_faces() == 0) {
		if (bezierTriangleMesh_.n_vertices() > numVerts_) {
			delete[] invVertexMap_;
			invVertexMap_ = 0;
		}
		numVerts_ = bezierTriangleMesh_.n_vertices();
		vertices_.resize(numVerts_ * vertexDecl_->getVertexStride());

		// read all vertices
		for (size_t i = 0; i < numVerts_; ++i)
			readVertex(i,
				bezierTriangleMesh_.vertex_handle(static_cast<unsigned int>(i)),
				(typename Mesh::HalfedgeHandle)(-1),
				(typename Mesh::FaceHandle)(-1));

		createVBO();
		rebuild_ = REBUILD_NONE;
		return;
	}
	*/
	invalidateFullVBO();

	unsigned int maxFaceVertCount = 0;
	unsigned int numIndices = 0;
	unsigned int newTriCount = countTris(&maxFaceVertCount, &numIndices);

	int bTriangleRebuild = 0; // what should be rebuild?
	int bVertexRebuild = 0;

	if (newTriCount > numTris_) {
		// index buffer too small
		deleteIbo();

		numTris_ = newTriCount;

		bTriangleRebuild = 1;
	}

	if (prevNumFaces_ != bezierTriangleMesh_.n_faces()) {
		bTriangleRebuild = 1;
		prevNumFaces_ = bezierTriangleMesh_.n_faces();
	}

	if (prevNumVerts_ != bezierTriangleMesh_.n_vertices()) {
		if (prevNumVerts_ < bezierTriangleMesh_.n_vertices()) {
			// resize inverse vertex map
			delete[] invVertexMap_;
			invVertexMap_ = new unsigned int[bezierTriangleMesh_.n_vertices()];
		}

		bVertexRebuild = 1;
		bTriangleRebuild = 1; // this may have caused changes in the topology!
		prevNumVerts_ = bezierTriangleMesh_.n_vertices();
	}

	// support faster update by only updating vertices (do a complete update if the textures have to be rebuild)
	if (!bTriangleRebuild && !bVertexRebuild && (rebuild_ & REBUILD_GEOMETRY) && !(rebuild_ & REBUILD_TEXTURES)) {
		// only update vertices, i.e. update values of vertices

#ifndef WIN32
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
#endif
		for (size_t i = 0; i < numVerts_; ++i) {
			// just pick one face, srews up face colors here so color updates need a full rebuild
			const auto hh = mapToHalfedgeHandle(i);
			typename BezierTMesh::VertexHandle   vh(-1);
			typename BezierTMesh::FaceHandle     fh(-1);

			if (hh.is_valid()) {
				vh = bezierTriangleMesh_.to_vertex_handle(hh);
				fh = bezierTriangleMesh_.face_handle(hh);
			} else {
				int f_id, c_id;
				int posID = meshComp_->mapToOriginalVertexID(i, f_id, c_id);
				vh = bezierTriangleMesh_.vertex_handle(posID);
			}

			readVertex(i, vh, hh, fh);
		}

		createVBO();

		rebuild_ = REBUILD_NONE;
		return;
	}


	// full rebuild:
	delete meshComp_;
	meshComp_ = new MeshCompiler(*vertexDecl_);

	// search for convenient attribute indices
	int attrIDNorm = -1, attrIDPos = -1, attrIDTexC = -1;

	for (int i = 0; i < (int)meshComp_->getVertexDeclaration()->getNumElements(); ++i) {
		const VertexElement* e = meshComp_->getVertexDeclaration()->getElement(i);

		switch (e->usage_) {
			case VERTEX_USAGE_POSITION: attrIDPos = i; break;
			case VERTEX_USAGE_NORMAL: attrIDNorm = i; break;
			case VERTEX_USAGE_TEXCOORD: attrIDTexC = i; break;
			default: break;
		}
	}


	// pass face data to mesh compiler
	DrawMeshFaceInput<BezierTMesh>* faceInput = new DrawMeshFaceInput<BezierTMesh>(bezierTriangleMesh_);
	faceInput->attributeStoredPerHalfedge_.resize(meshComp_->getVertexDeclaration()->getNumElements(), 0);
	faceInput->attributeStoredPerHalfedge_[attrIDPos] = 0;
	faceInput->attributeStoredPerHalfedge_[attrIDNorm] = ((halfedgeNormalMode_ && bezierTriangleMesh_.has_halfedge_normals()) ? 1 : 0);
	faceInput->attributeStoredPerHalfedge_[attrIDTexC] = (bezierTriangleMesh_.has_halfedge_texcoords2D() ? 1 : 0);

	// index source for custom attributes
	for (size_t i = 0; i < additionalElements_.size(); ++i) {
		const VertexProperty* prop = &additionalElements_[i];

		if (prop->declElementID_ >= 0)
			faceInput->attributeStoredPerHalfedge_[prop->declElementID_] = (prop->source_ == PROPERTY_SOURCE_HALFEDGE) ? 1 : 0;
	}

	meshComp_->setFaceInput(faceInput);

	// set textures
	for (unsigned int i = 0; i < bezierTriangleMesh_.n_faces(); ++i)
		meshComp_->setFaceGroup(i, getTextureIDofFace(i));

	// pass vertex data to mesh compiler
	// points
	meshComp_->setVertices(bezierTriangleMesh_.n_vertices(), bezierTriangleMesh_.points(), 24, false, GL_DOUBLE, 3);

	// normals
	if (halfedgeNormalMode_ && bezierTriangleMesh_.has_halfedge_normals())
		meshComp_->setNormals(bezierTriangleMesh_.n_halfedges(), bezierTriangleMesh_.property(bezierTriangleMesh_.halfedge_normals_pph()).data(), 24, false, GL_DOUBLE, 3);
	else if (bezierTriangleMesh_.has_vertex_normals())
		meshComp_->setNormals(bezierTriangleMesh_.n_vertices(), bezierTriangleMesh_.vertex_normals(), 24, false, GL_DOUBLE, 3);

	if (bezierTriangleMesh_.has_halfedge_texcoords2D())
		meshComp_->setTexCoords(bezierTriangleMesh_.n_halfedges(), bezierTriangleMesh_.htexcoords2D(), 8, false, GL_FLOAT, 2);

	// add more requested custom attribtues to mesh compiler here..

	for (size_t i = 0; i < additionalElements_.size(); ++i) {
		VertexProperty* propDesc = &additionalElements_[i];

		if (propDesc->declElementID_ >= 0) {
			const VertexElement* el = vertexDecl_->getElement((unsigned int)propDesc->declElementID_);

			if (el->usage_ == VERTEX_USAGE_SHADER_INPUT) {
				// get openmesh property handle
				OpenMesh::BaseProperty* baseProp = 0;

				switch (propDesc->source_) {
					case PROPERTY_SOURCE_VERTEX: baseProp = bezierTriangleMesh_._get_vprop(propDesc->name_); break;
					case PROPERTY_SOURCE_FACE: baseProp = bezierTriangleMesh_._get_fprop(propDesc->name_); break;
					case PROPERTY_SOURCE_HALFEDGE: baseProp = bezierTriangleMesh_._get_hprop(propDesc->name_); break;
					default: baseProp = bezierTriangleMesh_._get_vprop(propDesc->name_); break;
				}

				if (baseProp) {
					size_t numAttribs = baseProp->n_elements();
					const void* attribData = propDesc->propDataPtr_;

					meshComp_->setAttribVec(propDesc->declElementID_, numAttribs, attribData);
				}
			}
		}
	}

	// compile draw buffers
	meshComp_->build(true, true, true, true);


	// create inverse vertex map
	for (int i = 0; i < (int)bezierTriangleMesh_.n_faces(); ++i) {
		auto fh = bezierTriangleMesh_.face_handle(i);

		int corner = 0;

		for (auto hh_it = bezierTriangleMesh_.fh_iter(fh); hh_it.is_valid(); ++hh_it) {
			int vertexId = bezierTriangleMesh_.to_vertex_handle(*hh_it).idx();
			invVertexMap_[vertexId] = meshComp_->mapToDrawVertexID(i, corner++);
		}
	}


	// get vertex buffer
	numTris_ = meshComp_->getNumTriangles();
	numVerts_ = meshComp_->getNumVertices();

	vertices_.resize(numVerts_ * vertexDecl_->getVertexStride());
	meshComp_->getVertexBuffer(&vertices_[0]);

	// copy colors
	for (int i = 0; i < (int)numVerts_; ++i) {
		auto hh = mapToHalfedgeHandle(i);

		unsigned int col = 0;

		if (hh.is_valid())
			col = getVertexColor(bezierTriangleMesh_.to_vertex_handle(hh));
		else {
			// isolated vertex
			int f_id, c_id;
			int posID = meshComp_->mapToOriginalVertexID(i, f_id, c_id);
			col = getVertexColor(bezierTriangleMesh_.vertex_handle(posID));
		}

		writeColor(i, col);
	}

	// vbo stores per vertex colors
	curVBOColorMode_ = 1;

	// copy face colors to provoking id
	if (colorMode_ == 2) {
		const int provokingId = meshComp_->getProvokingVertex();
		assert(provokingId >= 0 && provokingId < 3);

		for (int i = 0; i < (int)numTris_; ++i) {
			int idx = meshComp_->getIndex(i * 3 + provokingId);

			int faceId = meshComp_->mapToOriginalFaceID(i);
			unsigned int fcolor = getFaceColor(bezierTriangleMesh_.face_handle(faceId));

			writeColor(idx, fcolor);
		}

#ifdef _DEBUG
		// debug check

		for (int i = 0; i < (int)numTris_; ++i) {
			int idx = meshComp_->getIndex(i * 3 + provokingId);

			int faceId = meshComp_->mapToOriginalFaceID(i);
			unsigned int fcolor = getFaceColor(bezierTriangleMesh_.face_handle(faceId));

			unsigned int storedColor = *(unsigned int*)(&vertices_[idx * vertexDecl_->getVertexStride() + offsetColor_]);

			if (storedColor != fcolor) {
				std::cout << "warning: possibly found provoking vertex shared by more than one face, writing report to ../../meshcomp_provoking.txt" << std::endl;

				// could also be caused by multi-threading, where one thread calls rebuild()
				// and the other thread updates face colors between previous for-loop and debug-check

				// check for errors
				meshComp_->dbgVerify("../../meshcomp_provoking.txt");

				break; // verify and dump report only once
			}
		}
#endif // _DEBUG

		curVBOColorMode_ = colorMode_;
	}

	//////////////////////////////////////////////////////////////////////////
	// copy to GPU

	createVBO();
	createIBO();

	bVBOinHalfedgeNormalMode_ = halfedgeNormalMode_;

	rebuild_ = REBUILD_NONE;
}

//========================================================================
// Setup vertex Declaration
//========================================================================

void DrawBTMesh::createVertexDeclaration()
{
	vertexDecl_->clear();

	vertexDecl_->addElement(GL_FLOAT, 3, VERTEX_USAGE_POSITION);
	vertexDecl_->addElement(GL_FLOAT, 2, VERTEX_USAGE_TEXCOORD);
	vertexDecl_->addElement(GL_FLOAT, 3, VERTEX_USAGE_NORMAL);
	vertexDecl_->addElement(GL_UNSIGNED_BYTE, 4, VERTEX_USAGE_COLOR);

	for (size_t i = 0; i < additionalElements_.size(); ++i) {
		VertexProperty* prop = &additionalElements_[i];

		// invalidate detected type
		prop->sourceType_.numElements_ = 0;
		prop->sourceType_.pointer_ = 0;
		prop->sourceType_.type_ = 0;
		prop->sourceType_.shaderInputName_ = 0;
		prop->sourceType_.usage_ = VERTEX_USAGE_SHADER_INPUT;
		prop->propDataPtr_ = 0;
		prop->declElementID_ = -1;

		// get property handle in openmesh by name
		OpenMesh::BaseProperty* baseProp = 0;

		switch (prop->source_) {
			case PROPERTY_SOURCE_VERTEX: baseProp = bezierTriangleMesh_._get_vprop(prop->name_); break;
			case PROPERTY_SOURCE_FACE: baseProp = bezierTriangleMesh_._get_fprop(prop->name_); break;
			case PROPERTY_SOURCE_HALFEDGE: baseProp = bezierTriangleMesh_._get_hprop(prop->name_); break;
			default: baseProp = bezierTriangleMesh_._get_vprop(prop->name_); break;
		}

		// detect data type of property
		prop->propDataPtr_ = getMeshPropertyType(baseProp, &prop->sourceType_.type_, &prop->sourceType_.numElements_);


		if (prop->propDataPtr_) {
			prop->sourceType_.shaderInputName_ = prop->name_.c_str();

			// should have same type in vbo
			prop->destType_ = prop->sourceType_;

			prop->destType_.shaderInputName_ = prop->vertexShaderInputName_.c_str();

			prop->declElementID_ = int(vertexDecl_->getNumElements());

			vertexDecl_->addElement(&prop->destType_);
		} else
			std::cerr << "Could not detect data type of property " << prop->name_ << std::endl;
	}
}

const void* DrawBTMesh::getMeshPropertyType(OpenMesh::BaseProperty* _prop, GLuint* _outType, unsigned int* _outSize) const
{
	const void* dataPtr = 0;

	// try float
	dataPtr = testMeshPropertyTypeT<float>(_prop, _outSize);
	if (dataPtr) {
		if (_outType) *_outType = GL_FLOAT;
		return dataPtr;
	}

	// try byte
	dataPtr = testMeshPropertyTypeT<char>(_prop, _outSize);
	if (dataPtr) {
		if (_outType) *_outType = GL_BYTE;
		return dataPtr;
	}

	// try ubyte
	dataPtr = testMeshPropertyTypeT<unsigned char>(_prop, _outSize);
	if (dataPtr) {
		if (_outType) *_outType = GL_UNSIGNED_BYTE;
		return dataPtr;
	}

	// try double
	dataPtr = testMeshPropertyTypeT<double>(_prop, _outSize);

	if (dataPtr) {
		if (_outType) *_outType = GL_DOUBLE;
		return dataPtr;
	}

	// try int
	dataPtr = testMeshPropertyTypeT<int>(_prop, _outSize);

	if (dataPtr) {
		if (_outType) *_outType = GL_INT;
		return dataPtr;
	}

	// try uint
	dataPtr = testMeshPropertyTypeT<unsigned int>(_prop, _outSize);

	if (dataPtr) {
		if (_outType) *_outType = GL_UNSIGNED_INT;
		return dataPtr;
	}

	// try short
	dataPtr = testMeshPropertyTypeT<short>(_prop, _outSize);

	if (dataPtr) {
		if (_outType) *_outType = GL_SHORT;
		return dataPtr;
	}

	// try ushort
	dataPtr = testMeshPropertyTypeT<unsigned short>(_prop, _outSize);

	if (dataPtr) {
		if (_outType) *_outType = GL_UNSIGNED_SHORT;
		return dataPtr;
	}


	// unknown data type
	if (_outSize)
		*_outSize = 0;

	if (_outType)
		*_outType = 0;

	return 0;
}

template<class T>
const void* DrawBTMesh::testMeshPropertyTypeT(const OpenMesh::BaseProperty* _prop, unsigned int* _outSize) const
{
	if (_outSize)
		*_outSize = 0;
	const void* dataPtr = 0;

	// rtti - detect type of property from openmesh via dynamic_cast
	typedef OpenMesh::PropertyT< T > Prop1;
	typedef OpenMesh::PropertyT< OpenMesh::VectorT<T, 1> > PropVec1;
	typedef OpenMesh::PropertyT< OpenMesh::VectorT<T, 2> > PropVec2;
	typedef OpenMesh::PropertyT< OpenMesh::VectorT<T, 3> > PropVec3;
	typedef OpenMesh::PropertyT< OpenMesh::VectorT<T, 4> > PropVec4;

	const Prop1* p1 = dynamic_cast<const Prop1*>(_prop);
	const PropVec1* pv1 = dynamic_cast<const PropVec1*>(_prop);
	const PropVec2* pv2 = dynamic_cast<const PropVec2*>(_prop);
	const PropVec3* pv3 = dynamic_cast<const PropVec3*>(_prop);
	const PropVec4* pv4 = dynamic_cast<const PropVec4*>(_prop);

	if (p1 || pv1) {
		if (_outSize)
			*_outSize = 1;
		if (p1)
			dataPtr = p1->data();
		else
			dataPtr = pv1->data();
	} else if (pv2) {
		if (_outSize)
			*_outSize = 2;
		dataPtr = pv2->data();
	} else if (pv3) {
		if (_outSize)
			*_outSize = 3;
		dataPtr = pv3->data();
	} else if (pv4) {
		if (_outSize)
			*_outSize = 4;
		dataPtr = pv4->data();
	}

	return dataPtr;
}

//========================================================================
// Setup VBO / IBO
//========================================================================

void DrawBTMesh::createVBO()
{
	bindVbo();

	// toggle between normal source and texcoord source
	// (per vertex, per halfedge, per face)

	if (flatMode_ && meshComp_) {
		for (unsigned int i = 0; i < numTris_; ++i) {
			int faceId = meshComp_->mapToOriginalFaceID(i);

			// get face normal
			ACG::Vec3d n = bezierTriangleMesh_.normal(bezierTriangleMesh_.face_handle(faceId));

			{
				int idx = meshComp_->getIndex(i * 3 + meshComp_->getProvokingVertex());
				writeNormal(idx, n);
			}

		}
		bVBOinFlatMode_ = 1;
	} else {
		for (unsigned int i = 0; i < numVerts_; ++i) {
			auto hh = mapToHalfedgeHandle(i);

			// get halfedge normal

			if (hh.is_valid()) {
				ACG::Vec3d n;
				if (halfedgeNormalMode_ == 1 && bezierTriangleMesh_.has_halfedge_normals())
					n = bezierTriangleMesh_.normal(hh);
				else
					n = bezierTriangleMesh_.normal(bezierTriangleMesh_.to_vertex_handle(hh));

				writeNormal(i, n);
			} else {
				// isolated vertex
				int posID = i;


				if (meshComp_) {
					int f_id, c_id;
					posID = meshComp_->mapToOriginalVertexID(i, f_id, c_id);
				}

				writeNormal(i, bezierTriangleMesh_.normal(bezierTriangleMesh_.vertex_handle(posID)));
			}
		}

		bVBOinFlatMode_ = 0;
	}

	if (textureMode_ == 0) {
		// per vertex texcoords
		if (bezierTriangleMesh_.has_vertex_texcoords2D()) {
			for (unsigned int i = 0; i < numVerts_; ++i) {
				auto hh = mapToHalfedgeHandle(i);

				if (hh.is_valid()) {
					writeTexcoord(i, bezierTriangleMesh_.texcoord2D(bezierTriangleMesh_.to_vertex_handle(hh)));
				} else {
					// isolated vertex
					int posID = i;

					if (meshComp_) {
						int f_id, c_id;
						posID = meshComp_->mapToOriginalVertexID(i, f_id, c_id);
					}

					writeTexcoord(i, bezierTriangleMesh_.texcoord2D(bezierTriangleMesh_.vertex_handle(posID)));
				}
			}
		}

		bVBOinHalfedgeTexMode_ = 0;
	} else {
		if (bezierTriangleMesh_.has_vertex_texcoords2D() || bezierTriangleMesh_.has_halfedge_texcoords2D()) {
			// per halfedge texcoords
			for (unsigned int i = 0; i < numVerts_; ++i) {
				auto hh = mapToHalfedgeHandle(i);

				if (hh.is_valid()) {
					// copy texcoord
					if (bezierTriangleMesh_.has_halfedge_texcoords2D()) {
						writeTexcoord(i, bezierTriangleMesh_.texcoord2D(hh));
					}

				} else if (bezierTriangleMesh_.has_vertex_texcoords2D()) {
					// isolated vertex
					int posID = i;

					if (meshComp_) {
						int f_id, c_id;
						posID = meshComp_->mapToOriginalVertexID(i, f_id, c_id);
					}
					writeTexcoord(i, bezierTriangleMesh_.texcoord2D(bezierTriangleMesh_.vertex_handle(posID)));
				}
			}
		}

		bVBOinHalfedgeTexMode_ = 1;
	}

	if (colorMode_ && colorMode_ != curVBOColorMode_) {
		if (colorMode_ == 1) {
			// use vertex colors

			for (int i = 0; i < (int)numVerts_; ++i) {
				auto hh = mapToHalfedgeHandle(i);

				unsigned int col;

				if (hh.is_valid())
					col = getVertexColor(bezierTriangleMesh_.to_vertex_handle(hh));
				else {
					// isolated vertex
					int f_id, c_id;
					int posID = meshComp_->mapToOriginalVertexID(i, f_id, c_id);
					col = getVertexColor(bezierTriangleMesh_.vertex_handle(posID));
				}

				writeColor(i, col);
			}
		} else if (colorMode_ == 2) {
			// use face colors

			const int provokingId = meshComp_->getProvokingVertex();
			assert(provokingId >= 0 && provokingId < 3);

			for (int i = 0; i < (int)numTris_; ++i) {
				int idx = meshComp_->getIndex(i * 3 + provokingId);

				int faceId = meshComp_->mapToOriginalFaceID(i);
				unsigned int fcolor = getFaceColor(bezierTriangleMesh_.face_handle(faceId));

				//        vertices_[idx].col = fcolor;
				writeColor(idx, fcolor);
			}
		}

		// vbo colors updated
		curVBOColorMode_ = colorMode_;
	}

	fillVertexBuffer();

	ACG::GLState::bindBufferARB(GL_ARRAY_BUFFER_ARB, 0);

	// non indexed vbo needs updating now
	invalidateFullVBO();
}

void DrawBTMesh::createIBO()
{
	// data read from indices_

	bindIbo();

	indexType_ = GL_UNSIGNED_INT;
	createIndexBuffer();


	// line index buffer:
	if (bezierTriangleMesh_.n_edges()) {
		std::vector<unsigned int> lineBuffer(bezierTriangleMesh_.n_edges() * 2);

		for (unsigned int i = 0; i < bezierTriangleMesh_.n_edges(); ++i) {
			OpenMesh::HalfedgeHandle hh = bezierTriangleMesh_.halfedge_handle(bezierTriangleMesh_.edge_handle(i), 0);

			if (indexType_ == GL_UNSIGNED_SHORT) {
				// put two words in a dword
				unsigned int combinedIdx = invVertexMap_[bezierTriangleMesh_.from_vertex_handle(hh).idx()] | (invVertexMap_[bezierTriangleMesh_.to_vertex_handle(hh).idx()] << 16);
				lineBuffer[i] = combinedIdx;
			} else {
				lineBuffer[2 * i] = invVertexMap_[bezierTriangleMesh_.from_vertex_handle(hh).idx()];
				lineBuffer[2 * i + 1] = invVertexMap_[bezierTriangleMesh_.to_vertex_handle(hh).idx()];
			}
		}

		bindLineIbo();

		fillLineBuffer(bezierTriangleMesh_.n_edges(), &lineBuffer[0]);
	}

	ACG::GLState::bindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
}

//========================================================================
// Utility
//========================================================================

unsigned int DrawBTMesh::countTris(unsigned int* pMaxVertsOut, unsigned int* _pOutNumIndices)
{
	unsigned int triCounter = 0;

	if (pMaxVertsOut) *pMaxVertsOut = 0;
	if (_pOutNumIndices) *_pOutNumIndices = 0;

	for (unsigned int i = 0; i < bezierTriangleMesh_.n_faces(); ++i) {
		auto fh = bezierTriangleMesh_.face_handle(i);

		// count vertices
		unsigned int nPolyVerts = 0;

		for (auto hh_it = bezierTriangleMesh_.fh_iter(fh); hh_it.is_valid(); ++hh_it) ++nPolyVerts;

		triCounter += (nPolyVerts - 2);

		if (pMaxVertsOut) {
			if (*pMaxVertsOut < nPolyVerts)
				*pMaxVertsOut = nPolyVerts;
		}

		if (_pOutNumIndices) *_pOutNumIndices += nPolyVerts;
	}

	return triCounter;
}

BezierTMesh::HalfedgeHandle DrawBTMesh::mapToHalfedgeHandle(size_t _vertexId)
{
	int faceId = -1, cornerId = -1;

	// map to halfedge handle
	if (meshComp_)
		meshComp_->mapToOriginalVertexID(_vertexId, faceId, cornerId);

	if (faceId >= 0) {
		auto fh = bezierTriangleMesh_.face_handle(faceId);
		auto hh_it = bezierTriangleMesh_.fh_iter(fh);

		// seek to halfedge
		for (int k = 0; k < cornerId && hh_it.is_valid(); ++k)
			++hh_it;

		return *hh_it;
	} else
		return BezierTMesh::HalfedgeHandle(-1);
}

void DrawBTMesh::readVertex(
	size_t _vertex,
	const BezierTMesh::VertexHandle&   _vh,
	const BezierTMesh::HalfedgeHandle& _hh,
	const BezierTMesh::FaceHandle&     _fh
)
{
	static const BezierTMesh::HalfedgeHandle invalidHEH(-1);
	static const BezierTMesh::FaceHandle     invalidFH(-1);


	ACG::Vec3d n(0.0, 0.0, 1.0);
	ACG::Vec2f texc(0.0f, 0.0f);
	unsigned int col(0);

	// read normal
	if (halfedgeNormalMode_ == 0 && bezierTriangleMesh_.has_vertex_normals())
		n = bezierTriangleMesh_.normal(_vh);
	else if (halfedgeNormalMode_ &&  bezierTriangleMesh_.has_halfedge_normals() && _hh != invalidHEH)
		n = bezierTriangleMesh_.normal(_hh);

	// read texcoord
	if (bezierTriangleMesh_.has_halfedge_texcoords2D()) {
		if (_hh != invalidHEH && textureMode_ == 1)
			texc = bezierTriangleMesh_.texcoord2D(_hh);
		else if (bezierTriangleMesh_.has_vertex_texcoords2D())
			texc = bezierTriangleMesh_.texcoord2D(_vh);
	} else if (bezierTriangleMesh_.has_vertex_texcoords2D())
		texc = bezierTriangleMesh_.texcoord2D(_vh);

	// read per face or per vertex color
	unsigned int byteCol[2];
	for (int col = 0; col < 2; ++col) {
		Vec4uc vecCol(255, 255, 255, 255);

		if (col == 0 && bezierTriangleMesh_.has_vertex_colors())
			vecCol = OpenMesh::color_cast<Vec4uc, BezierTMesh::Color>(bezierTriangleMesh_.color(_vh));
		if (_fh != invalidFH) {
			if (col == 1 && bezierTriangleMesh_.has_face_colors() && _fh.idx() >= 0)
				vecCol = OpenMesh::color_cast<Vec4uc, BezierTMesh::Color>(bezierTriangleMesh_.color(_fh));
		}

		// OpenGL color format: A8B8G8R8
		byteCol[col] = (unsigned char)(vecCol[0]);
		byteCol[col] |= ((unsigned char)(vecCol[1])) << 8;
		byteCol[col] |= ((unsigned char)(vecCol[2])) << 16;
		byteCol[col] |= ((unsigned char)(vecCol[3])) << 24;
		//byteCol[col] |= 0xFF << 24; // if no alpha channel
	}

	if (colorMode_ != 2)
		col = byteCol[0]; // vertex colors
	else
		col = byteCol[1]; // face colors


	// store vertex attributes in vbo
	writePosition(_vertex, bezierTriangleMesh_.point(_vh));
	writeNormal(_vertex, n);
	writeTexcoord(_vertex, texc);
	writeColor(_vertex, col);


	// read/write custom attributes
	for (size_t i = 0; i < additionalElements_.size(); ++i) {
		std::cout << "not implemented!" << std::endl;
	}
}

void DrawBTMesh::writeVertexElement(void* _dstBuf, size_t _vertex, size_t _stride, size_t _elementOffset, size_t _elementSize, const void* _elementData)
{
	// byte offset
	size_t offset = _vertex * _stride + _elementOffset;

	// write address
	char* dst = static_cast<char*>(_dstBuf) + offset;

	// copy
	memcpy(dst, _elementData, _elementSize);
}

void DrawBTMesh::writePosition(size_t _vertex, const ACG::Vec3d& _n)
{
	// store float3 position
	float f3[3] = { float(_n[0]), float(_n[1]), float(_n[2]) };

	writeVertexElement(&vertices_[0], _vertex, vertexDecl_->getVertexStride(), 0, 12, f3);
}

void DrawBTMesh::writeNormal(size_t _vertex, const ACG::Vec3d& _n)
{
	// store float3 normal
	float f3[3] = { float(_n[0]), float(_n[1]), float(_n[2]) };

	writeVertexElement(&vertices_[0], _vertex, vertexDecl_->getVertexStride(), offsetNormal_, 12, f3);
}

void DrawBTMesh::writeTexcoord(size_t _vertex, const ACG::Vec2f& _uv)
{
	writeVertexElement(&vertices_[0], _vertex, vertexDecl_->getVertexStride(), offsetTexc_, 8, _uv.data());
}

void DrawBTMesh::writeColor(size_t _vertex, unsigned int _color)
{
	writeVertexElement(&vertices_[0], _vertex, vertexDecl_->getVertexStride(), offsetColor_, 4, &_color);
}

unsigned int DrawBTMesh::getVertexColor(const BezierTMesh::VertexHandle& _vh)
{
	static const BezierTMesh::VertexHandle invalidVH(-1);

	unsigned int byteCol;

	Vec4uc vecCol(255, 255, 255, 255);

	if (_vh != invalidVH && bezierTriangleMesh_.has_vertex_colors())
		vecCol = OpenMesh::color_cast<Vec4uc, BezierTMesh::Color>(bezierTriangleMesh_.color(_vh));

	// OpenGL color format: A8B8G8R8
	byteCol = (unsigned char)(vecCol[0]);
	byteCol |= ((unsigned char)(vecCol[1])) << 8;
	byteCol |= ((unsigned char)(vecCol[2])) << 16;
	byteCol |= ((unsigned char)(vecCol[3])) << 24;

	return byteCol;
}

unsigned int DrawBTMesh::getFaceColor(const BezierTMesh::FaceHandle& _fh)
{
	static const BezierTMesh::FaceHandle invalidFH(-1);

	unsigned int byteCol;
	Vec4uc vecCol(255, 255, 255, 255);

	if (_fh != invalidFH && bezierTriangleMesh_.has_face_colors() && _fh.idx() >= 0)
		vecCol = OpenMesh::color_cast<Vec4uc, BezierTMesh::Color>(bezierTriangleMesh_.color(_fh));

	// OpenGL color format: A8B8G8R8
	byteCol = (unsigned char)(vecCol[0]);
	byteCol |= ((unsigned char)(vecCol[1])) << 8;
	byteCol |= ((unsigned char)(vecCol[2])) << 16;
	byteCol |= ((unsigned char)(vecCol[3])) << 24;

	return byteCol;
}

//========================================================================
// texture handling
//========================================================================

int DrawBTMesh::getTextureIDofFace(unsigned int _face)
{
	OpenMesh::FPropHandleT< int > textureIndexProperty;
	if (bezierTriangleMesh_.get_property_handle(textureIndexProperty, textureIndexPropertyName_))
		return bezierTriangleMesh_.property(textureIndexProperty, bezierTriangleMesh_.face_handle(_face));

	if (bezierTriangleMesh_.has_face_texture_index())
		return bezierTriangleMesh_.texture_index(bezierTriangleMesh_.face_handle(_face));

	return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Tesselation
///////////////////////////////////////////////////////////////////////////////

/**
 * This method is only evaluated if the Mesh was changed (invalidateSurfaceMesh_)
 * This is the case if updateGeometry() is called.
 */
void DrawBTMesh::updateSurfaceMesh(const int meshOption)
{
	if (!invalidateSurfaceMesh_)
		return;

	surfaceVBO_.del();
	surfaceIBO_.del();

	// vertex layout:
	//  float3 pos
	//  float3 normal
	//  float2 texcoord
	//  + debug info (optional)

	// provide expected values of bspline evaluation steps for debugging in shader
	const bool provideDebugInfo = false;


	if (!surfaceDecl_.getNumElements()) {
		surfaceDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_POSITION);
		surfaceDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_NORMAL);
		surfaceDecl_.addElement(GL_FLOAT, 2, VERTEX_USAGE_TEXCOORD);
		//surfaceDecl_.addElement(GL_FLOAT, 4, VERTEX_USAGE_COLOR);
		//surfaceDecl_.addElement(GL_UNSIGNED_BYTE, 4, VERTEX_USAGE_COLOR); TODO

		if (provideDebugInfo) {
			surfaceDecl_.addElement(GL_FLOAT, 2, VERTEX_USAGE_SHADER_INPUT, size_t(0), "a2v_span");
			surfaceDecl_.addElement(GL_FLOAT, 4, VERTEX_USAGE_SHADER_INPUT, size_t(0), "a2v_bvu");
			surfaceDecl_.addElement(GL_FLOAT, 4, VERTEX_USAGE_SHADER_INPUT, size_t(0), "a2v_bvv");
		}
	}

	if (false) { // TODO if apply tesselation - should get a separate call
		// TODO
		// TODO should the mesh really be changed? we could simple apply the
		// changes to the vbo and dont change the Mesh itself
		// TODO Button für applyTesselation
		//tesselateMeshCPU();
	}

	std::cerr << " bla " << std::endl; // TODO
	// TODO Performance verbessern indem in den vertex buffer alle vertices gepackt werden und dann
	// beim index buffer die indices direkt genutzt werden

	// BIG TODO !!!
	// not really sure what happens - but the renderOption should
	// decide if there is CPU or GPU tesselation (or both), the render mode needs to
	// change based on that
	// Generate a VBO from the Mesh without CPU tesselation
	if (meshOption == betri::TESSELLATION_TYPE::GPU || meshOption == betri::TESSELLATION_TYPE::NONE) {
		VBOfromMesh();
	}
	// Generate a VBO and apply CPU tesselation without changing the Mesh
	else if (meshOption == betri::TESSELLATION_TYPE::CPU) {
		std::cerr << " bla2  " << std::endl; // TODO
		VBOtesselatedFromMesh();
	} else if (meshOption == betri::TESSELLATION_TYPE::RAYTRACING) {
		VBOfromBoundingMesh();
	}
}

int DrawBTMesh::pointsBefore(int level)
{
	// TODO das ist langsam?!
	int sum = 0;
	for (int i = 0; i < level; i++) {
		sum += GRAD + 1 - i;
	}
	return sum;
}

BezierTMesh::Point DrawBTMesh::getCP(
	int i, int j, int k, BezierTMesh::FaceHandle fh
)
{
	int cpIndex = pointsBefore(i) + j;

	auto faceControlP = bezierTriangleMesh_.data(fh);
	return faceControlP.controlPoint(cpIndex);
}

BezierTMesh::Point DrawBTMesh::oneEntry(
	int i, int j, int k,
	BezierTMesh::Point baryCoords, BezierTMesh::FaceHandle fh
)
{
	// TODO
	const float FACTORIALS[13] = {
		1, 1, 2, 6, 24, 120, // 0, 1, 2, 3, 4, 5
		720, 5040, 40320, 362880, 3628800, // 6, 7, 8, 9, 10
		39916800, 479001600
	};

	Point entry = FACTORIALS[GRAD] / (FACTORIALS[i] * FACTORIALS[j] * FACTORIALS[k])
		* pow(baryCoords[0], i) * pow(baryCoords[1], j) * pow(baryCoords[2], k)
		* getCP(i, j, k, fh);
	return entry;
}

BezierTMesh::Point DrawBTMesh::newPosition(
	BezierTMesh::Point baryCoords, BezierTMesh::FaceHandle fh
)
{
	// 0 0 2
	// 0 1 1
	// 0 2 0
	// 1 0 1
	// 1 1 0
	// 2 0 0
	Point sum = Point(0.0);
	for (int i = 0; i <= GRAD; i++) {
		for (int j = 0; j + i <= GRAD; j++) {
			for (int k = GRAD - i - j; k + j + i == GRAD && k >= 0; k++) {
				sum += oneEntry(i, j, k, baryCoords, fh);
			}
		}
	}
	return sum;
}

/**
 * Create an VBO from the given Mesh object and apply CPU-tesselation
 * to the mesh without changing it representation as an BezierTriangleMesh.
 * (Only the VBO contains the additional vertex data)
 */
void DrawBTMesh::VBOtesselatedFromMesh()
{
	// TODO
	//oldFaceCount_ = bezierTriangleMesh_.n_faces();

	// create vertex buffer
	int vertexCount = bezierTriangleMesh_.n_faces() * VERTEXSUM;

	GLsizeiptr vboSize = vertexCount * surfaceDecl_.getVertexStride(); // bytes
	std::vector<float> vboData(vboSize); // float: 4 bytes

	// write counter
	int elementOffset = 0;

	int i = 0;
	Point pos;
	Point normal; //TODO BezierTMesh::Normal
	//Vec4uc vecCol(0, 0, 0, 1);
	Point vecCol(0, 0, 1);
	OpenMesh::VectorT<float, 2> texCoord; // TODO haben wir nicht brauchen wir noch
	for (auto &face : bezierTriangleMesh_.faces()) {
		auto vertexHandle = bezierTriangleMesh_.fv_begin(face);
		auto vh0 = *(vertexHandle++);
		auto vh1 = *(vertexHandle++);
		auto vh2 = *(vertexHandle++);

		for (double u = 0.0; u <= 1.0; u += STEPSIZE) {
			for (double v = 0.0; u + v <= 1.0; v += STEPSIZE) {

				// Get the 3D-position Barycentric coords
				auto toEval = betri::getBaryCoords(u, v);

				auto resultPoint = newPosition(toEval, face);

				// store pos
				pos = resultPoint;
				for (int m = 0; m < 3; ++m)
					vboData[elementOffset++] = float(pos[m]);

				// TODO ist das so richtig? welcher PUnkt ist der der mit u multipliziert werden muss?
				// TODO kreuzprodukt für normale?
				// store normal
				normal = bezierTriangleMesh_.normal(vh0) * u + bezierTriangleMesh_.normal(vh1) * v + bezierTriangleMesh_.normal(vh2) * (1 - u - v);
				for (int m = 0; m < 3; ++m)
					vboData[elementOffset++] = float(normal[m]);

				// store texcoord
				//texCoord = bezierTriangleMesh_.texcoord2D(v); // TODO
				//vboData[elementOffset++] = texCoord[0];
				//vboData[elementOffset++] = texCoord[1];
				vboData[elementOffset++] = 1.0;
				vboData[elementOffset++] = 0.0;

				/*
				// TODO use ints instead of floats
				//color = std::vector<float>({ 1.0, 0.0, 0.0, 1.0 });
				//color = bezierTriangleMesh_.texcoord2D(v);
				for (int m = 0; m < 3; ++m) {
					float y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
					vecCol[0] = y;
					vecCol[1] = y;
					vecCol[2] = y;
					auto x = bezierTriangleMesh_.color(face);
					std::cerr << x[0] << " " << x[1] << " " << x[2] << " " << y << " " << std::endl;
					vboData[elementOffset++] = float(vecCol[m]);
				}
				vboData[elementOffset++] = float(1.0);*/

				/* DrawMeshT<Mesh>::readVertex(
				byteCol[col] = (unsigned char)(vecCol[0]);
				byteCol[col] |= ((unsigned char)(vecCol[1])) << 8;
				byteCol[col] |= ((unsigned char)(vecCol[2])) << 16;
				byteCol[col] |= ((unsigned char)(vecCol[3])) << 24;
				*/
			}
		}
	}

	if (vboSize)
		surfaceVBO_.upload(vboSize, &vboData[0], GL_STATIC_DRAW);

	vboData.clear();

	// create index buffer
	//int numIndices = vboSize / 4;
	// TODO geht das besser?
	int numIndices = pow(4, ITERATIONS) * 3 * bezierTriangleMesh_.n_faces();
	std::vector<int> iboData(numIndices);

	int faceindex = 0, idxOffset = 0;
	for (auto &face : bezierTriangleMesh_.faces()) {

		int offset = faceindex * VERTEXSUM;
		int pos1 = offset + 0;
		int pos2 = offset + 1;
		int pos3 = offset + NEWVERTICES + 2;
		int border = offset + NEWVERTICES + 2;
		int boderAdd = NEWVERTICES + 2 - 1;

		for (; pos3 < (faceindex + 1) * VERTEXSUM; ) {
			iboData[idxOffset++] = pos1;
			iboData[idxOffset++] = pos2;
			iboData[idxOffset++] = pos3;

			if (pos2 + 1 < border) {
				// top triangle
				iboData[idxOffset++] = pos2;
				iboData[idxOffset++] = pos3 + 1;
				iboData[idxOffset++] = pos3;
			}

			if (pos2 + 1 == border) {
				border += boderAdd--;
				pos1++;
				pos2++;
			}

			pos1++;
			pos2++;
			pos3++;

		}

		faceindex++;
	}

	// TODO ist das hier in bytes und deswegen *4?
	if (numIndices)
		surfaceIBO_.upload(numIndices * 4, &iboData[0], GL_STATIC_DRAW);

	surfaceIndexCount_ = numIndices;

	invalidateSurfaceMesh_ = false;
}

/**
 * Create a simple VBO from this Mesh.
 */
void DrawBTMesh::VBOfromMesh()
{
	// create vertex buffer
	int vertexCount = bezierTriangleMesh_.n_faces() * 3;

	GLsizeiptr vboSize = vertexCount * surfaceDecl_.getVertexStride(); // bytes
	std::vector<float> vboData(vboSize); // float: 4 bytes

	// write counter
	int elementOffset = 0;

	int i = 0;
	Point pos;
	Point normal; //BezierTMesh::Normal
	OpenMesh::VectorT<float, 2> texCoord; // TODO haben wir nicht brauchen wir noch
	for (auto &face : bezierTriangleMesh_.faces()) {
		for (auto v = bezierTriangleMesh_.fv_begin(face); v != bezierTriangleMesh_.fv_end(face); ++v) {
			// store pos
			pos = bezierTriangleMesh_.point(v);
			for (int m = 0; m < 3; ++m)
				vboData[elementOffset++] = float(pos[m]);

			// store normal
			normal = bezierTriangleMesh_.normal(v);
			for (int m = 0; m < 3; ++m)
				vboData[elementOffset++] = float(normal[m]);

			// store texcoord
			//texCoord = bezierTriangleMesh_.texcoord2D(v); // TODO
			//vboData[elementOffset++] = texCoord[0];
			//vboData[elementOffset++] = texCoord[1];
			vboData[elementOffset++] = 1.0;
			vboData[elementOffset++] = 0.0;
		}
	}

	if (vboSize)
		surfaceVBO_.upload(vboSize, &vboData[0], GL_STATIC_DRAW);

	vboData.clear();

	// create index buffer
	int numIndices = vboSize / 4; // TODO warum hier durch 4

	std::vector<int> iboData(numIndices);

	// index counter
	//int idxOffset = 0;

	for (int idxOffset = 0; idxOffset < numIndices; ++idxOffset) {
		iboData[idxOffset] = idxOffset;
	}

	// TODO i think the numIndices should be *4 and the numIndices-count itself is wrong, try to compare it with idxOffset
	if (numIndices)
		surfaceIBO_.upload(numIndices, &iboData[0], GL_STATIC_DRAW);

	surfaceIndexCount_ = numIndices;

	invalidateSurfaceMesh_ = false;
}

/**
 * Create a simple VBO from this Mesh.
 */
void DrawBTMesh::VBOfromBoundingMesh()
{
	///////////////////////////////////////////////////////////////////////////
	// Setup VBO and IBO
	///////////////////////////////////////////////////////////////////////////

	// TODO different bounding volumes
	int bVolume = betri::option(betri::BezierOption::BOUNDING_VOLUME);

	int numVerts;
	int numIndices;
	betri::getVertexIndexCounts(bVolume, numVerts, numIndices);

	int vertexCount = bezierTriangleMesh_.n_faces() * numVerts;
	GLsizeiptr vboSize = vertexCount * surfaceDecl_.getVertexStride(); // bytes

	int indexCount = bezierTriangleMesh_.n_faces() * numIndices;

	// create index buffer
	std::vector<int> iboData(indexCount);
	// create vertex buffer
	std::vector<float> vboData(vboSize / 4); // float: 4 bytes

	///////////////////////////////////////////////////////////////////////////
	// Fill with boundingbox data
	///////////////////////////////////////////////////////////////////////////

	int vboIndex = 0;
	int iboIndex = 0;
	//for (int face_index = 0; face_index < bezierTriangleMesh_.n_faces(); ++face_index) {
	int face_index = 0;

	for (auto &face : bezierTriangleMesh_.faces()) {

		auto faceControlP = bezierTriangleMesh_.data(face);
		std::vector<Point> cpArray = std::vector<Point>();
		for (int i = 0; i < controlPointsPerFace; i++) {
			cpArray.push_back(faceControlP.controlPoint(i));
		}

		switch (bVolume) {
			case betri::boundingVolumeType::AABB:
			{
				// TODO is this the correct way to call this?
				betri::addBoundingBoxFromPoints(
					controlPointsPerFace,
					vboIndex,
					iboIndex,
					face_index,
					vboData,
					iboData,
					cpArray
				);
				break;
			}
			case betri::boundingVolumeType::PrismVolume:
			{
				// TODO is this the correct way to call this?
				betri::addPrismVolumeFromPoints(
					controlPointsPerFace,
					GRAD,
					vboIndex,
					iboIndex,
					face_index,
					vboData,
					iboData,
					cpArray
				);
				break;
			}
			default:
			{
				// TODO is this the correct way to call this?
				betri::addBoundingBoxFromPoints(
					controlPointsPerFace,
					vboIndex,
					iboIndex,
					face_index,
					vboData,
					iboData,
					cpArray
				);
			}
		}

		face_index++;
	}

	///////////////////////////////////////////////////////////////////////////
	// Upload VBO and IBO and cleanup
	///////////////////////////////////////////////////////////////////////////

	if (vboSize)
		surfaceVBO_.upload(vboSize, &vboData[0], GL_STATIC_DRAW);

	vboData.clear();

	// TODO why is it here *4 is it because of size in bytes?!
	if (indexCount)
		surfaceIBO_.upload(indexCount * 4, &iboData[0], GL_STATIC_DRAW);

	surfaceIndexCount_ = indexCount;

	invalidateSurfaceMesh_ = false;

}

//=============================================================================
} // namespace betri
//} // namespace ACG
//=============================================================================