#define ACG_BEZIERTRIANGLEMESHNODE_C

///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////
#include "BezierTriangleMeshNode.hh"
#include <ACG/GL/gl.hh>
#include <ACG/GL/GLError.hh>
#include <ACG/GL/IRenderer.hh>
#include <ACG/Utils/VSToolsT.hh>
#include <vector>

#include <fstream> // TODO
#include <functional> // TODO

#include "globals/BezierOptions.hh"
#include "BezierMathUtil.hh"
#include "boundVol/BVolGenerator.hh"

///////////////////////////////////////////////////////////////////////////////
// Defines
///////////////////////////////////////////////////////////////////////////////
#define ITERATIONS betri::option(betri::BezierOption::TESSELLATION_AMOUNT)
//#define RENDER_DEBUG

///////////////////////////////////////////////////////////////////////////////
// Namespaces
///////////////////////////////////////////////////////////////////////////////
namespace ACG {
namespace SceneGraph {

///////////////////////////////////////////////////////////////////////////////
// Setup
///////////////////////////////////////////////////////////////////////////////

// TODO look if the order should be changed - see MeshNode2T_impl.hh for that
template <class MeshT>
void BezierTriangleMeshNode<MeshT>::boundingBox(Vec3d& _bbMin, Vec3d& _bbMax)
{
	if (!bezierTriangleMesh_.isRenderable()) return;
	// TODO
	//setControlPointsColumnwise();

	const int controlPointsPerFace = cpSum();
	for (auto face : bezierTriangleMesh_.faces()) {
		auto faceControlP = bezierTriangleMesh_.data(face);
		Point cp;
		for (int i = 0; i < controlPointsPerFace; i++) {
			cp = faceControlP.controlPoint(i);
			_bbMin.minimize(cp);
			_bbMax.maximize(cp);
		}
	}
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::setControlPointsCircular()
{
	for (auto &face : bezierTriangleMesh_.faces()) {
		auto vertexHandle = bezierTriangleMesh_.fv_begin(face);
		auto vh0 = *(vertexHandle++);
		auto vh1 = *(vertexHandle++);
		auto vh2 = *(vertexHandle);

		int testScalar = 0.0;
		auto cp0 = bezierTriangleMesh_.point(vh0);
		auto cp1 = (bezierTriangleMesh_.point(vh0) * 0.5 + bezierTriangleMesh_.point(vh1) * 0.5) + Point(0.0, 0.0, testScalar);
		auto cp2 = bezierTriangleMesh_.point(vh1);
		auto cp3 = (bezierTriangleMesh_.point(vh1) * 0.5 + bezierTriangleMesh_.point(vh2) * 0.5) + Point(0.0, 0.0, testScalar);
		auto cp4 = bezierTriangleMesh_.point(vh2);
		auto cp5 = bezierTriangleMesh_.point(vh2) * 0.5 + bezierTriangleMesh_.point(vh0) * 0.5 + Point(0.0, 0.0, testScalar);

		bezierTriangleMesh_.data(face).points(std::vector<Point>({ cp0, cp1, cp2, cp3, cp4, cp5 }));
	}
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::setControlPointsColumnwise()
{
	// TODO: rene/franzis toggle !!!
	//return;
	float div = 1.0 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (10.0 - 1.0)));

	// Columnwise
	for (auto &face : bezierTriangleMesh_.faces()) {
		auto vertexHandle = bezierTriangleMesh_.fv_begin(face);
		auto vh0 = *(vertexHandle++);
		auto vh1 = *(vertexHandle++);
		auto vh2 = *(vertexHandle);


		Point p0 = bezierTriangleMesh_.point(vh0);
		Point p1 = bezierTriangleMesh_.point(vh1);
		Point p2 = bezierTriangleMesh_.point(vh2);

		std::vector<Point> cp_vec = std::vector<Point>();

		// TODO 1.01 ...
		const float CP_STEPSIZE = 1.0 / grad();
		int i = 0;
		for (double u = 0.0; u <= 1.01; u += CP_STEPSIZE) {
			for (double v = 0.0; u + v <= 1.01; v += CP_STEPSIZE) {
				double w = 1 - u - v;
				Point p = p0 * u + p1 * v + p2 * w;
				// If it isnt an cornerpoint
				if (i != 0 && i != grad() && i != betri::gaussSum(grad() + 1) - 1) {
					Point n = bezierTriangleMesh_.normal(vh0) * u + bezierTriangleMesh_.normal(vh1) * v + bezierTriangleMesh_.normal(vh2) * w;
					p += n / div;
					//p += Point(0.3, 0.0, 0.0);
				}
				i++;
				cp_vec.push_back(p);
				//std::cerr << cp_vec.size() << " " << u << " " << v << " " << w << " " << std::endl;
			}
		}

		bezierTriangleMesh_.data(face).points(std::vector<Point>(cp_vec));
	}
}

//-----------------------------------------------------------------------------

template <class MeshT>
DrawModes::DrawMode BezierTriangleMeshNode<MeshT>::availableDrawModes() const
{
	DrawModes::DrawMode drawModes(DrawModes::NONE);

	// We can always render points and a wireframe.
	drawModes |= DrawModes::POINTS;
	drawModes |= DrawModes::HIDDENLINE;
	drawModes |= DrawModes::WIREFRAME;
	drawModes |= DrawModes::HALFEDGES;

	// TODO was das?
	drawModes |= DrawModes::SOLID_SHADER;

	if (bezierTriangleMesh_.has_vertex_normals()) {
		drawModes |= DrawModes::POINTS_SHADED;
		drawModes |= DrawModes::SOLID_SMOOTH_SHADED;
		drawModes |= DrawModes::SOLID_PHONG_SHADED;
	}

	if (bezierTriangleMesh_.has_face_normals()) {
		drawModes |= DrawModes::SOLID_FLAT_SHADED;
	}

	if (bezierTriangleMesh_.has_halfedge_normals()) {
		drawModes |= DrawModes::SOLID_SMOOTH_SHADED_FEATURES;
	}

	if (bezierTriangleMesh_.has_vertex_colors()) {
		drawModes |= DrawModes::POINTS_COLORED;
		drawModes |= DrawModes::SOLID_POINTS_COLORED;

		std::cerr << " ja 2" << std::endl;

		if (bezierTriangleMesh_.has_vertex_normals())
			drawModes |= DrawModes::SOLID_POINTS_COLORED_SHADED;
	}

	if (bezierTriangleMesh_.has_edge_colors()) {
		drawModes |= DrawModes::EDGES_COLORED;
	}

	if (bezierTriangleMesh_.has_halfedge_colors()) {
		drawModes |= DrawModes::HALFEDGES_COLORED;
	}

	// TODO
	//bool enableTexturedFaces = drawMesh_->perFaceTextureCoordinateAvailable() != 0;

	if (bezierTriangleMesh_.has_face_colors()) {
		drawModes |= DrawModes::SOLID_FACES_COLORED;

		std::cerr << " ja " << std::endl;

		if (bezierTriangleMesh_.has_face_normals())
			drawModes |= DrawModes::SOLID_FACES_COLORED_FLAT_SHADED;

		if (bezierTriangleMesh_.has_vertex_normals()) {
			drawModes |= DrawModes::SOLID_FACES_COLORED_SMOOTH_SHADED;

			//if (enableTexturedFaces)
			//	drawModes |= DrawModes::SOLID_FACES_COLORED_2DTEXTURED_FACE_SMOOTH_SHADED;
		}
	}

	if (bezierTriangleMesh_.has_vertex_texcoords2D()) {
		drawModes |= DrawModes::SOLID_TEXTURED;

		if (bezierTriangleMesh_.has_vertex_normals())
			drawModes |= DrawModes::SOLID_TEXTURED_SHADED;
	}

	/*
	if (enableTexturedFaces) {
		drawModes |= DrawModes::SOLID_2DTEXTURED_FACE;

		if (bezierTriangleMesh_.has_face_normals())
			drawModes |= DrawModes::SOLID_2DTEXTURED_FACE_SHADED;
	}
	*/

	return drawModes;
}

///////////////////////////////////////////////////////////////////////////////
// Draw Deferred Functions
///////////////////////////////////////////////////////////////////////////////

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::getRenderObjects(
	IRenderer* _renderer, GLState& _state,
	const DrawModes::DrawMode& _drawMode, const Material* _mat
)
{
	std::clock_t start = std::clock();

	// TODO this should propably be done differently
	state_ = _state;

	// only render mesh if that is possible (e.g. has control points)
	if (!bezierTriangleMesh_.isRenderable())
		return;

	//std::cerr << bezierTriangleMesh_.has_face_normals() << " " << bezierTriangleMesh_.has_vertex_normals() << std::endl;
	// bezierTriangleMesh_.request_face_normals();

#define old
#ifndef old
	// TODO call otherwise or atleast only when needed
	drawBTMesh_->getVBO(); // TODO this should be done differently, in a status node or smthg look into that further
#endif

	// TODO
	if (controlPointsChangedR_) {
		setControlPointsColumnwise();
		controlPointsChangedR_ = false;
		controlPointsChangedC_ = true;
	}

	// check if textures are still valid
	if (bspline_selection_draw_mode_ == CONTROLPOINT
		&& controlPointSelectionTexture_valid_ == false)
		updateControlPointSelectionTexture(_state);
	if (bspline_selection_draw_mode_ == KNOTVECTOR
		&& knotVectorSelectionTexture_valid_ == false)
		updateKnotVectorSelectionTexture(_state);

	int renderOption = betri::option(betri::BezierOption::TESSELLATION_TYPE);
	int showBVolume = betri::option(betri::BezierOption::SHOW_BOUNDING_VOLUME);

	// TODO
	//updateSurfaceMesh(renderOption);


	RenderObject ro;
	ro.debugName = "BTMeshNode"; // TODO geht das so?

	for (size_t i = 0; i < _drawMode.getNumLayers(); ++i) {
		const DrawModes::DrawModeProperties* props = _drawMode.getLayer(i);

		//std::cerr << "IN MAIN LOOP: i " << i  << " isColored? " << props->colored() << std::endl;
		//std::cerr << "IN MAIN LOOP: colorSource " << props->colorSource() << std::endl;

		// TODO this can propably be done differently
		// TODO make this toggle
		//ACG::GLState::enable(GL_CULL_FACE);
		//ACG::GLState::cullFace(GL_FRONT);

		//std::cerr << bool(glIsEnabled(GL_CULL_FACE)) << " " << _state.isStateEnabled(GL_CULL_FACE) << std::endl;

		ro.initFromState(&_state);
		ro.setupShaderGenFromDrawmode(props);
		ro.depthTest = true;
#ifndef old
		///////////////////////////////////////////////////////////////////////
		// Setup drawMesh based on property source
		///////////////////////////////////////////////////////////////////////

		ro.shaderDesc.vertexColors = true;

		switch (props->colorSource()) {
			case DrawModes::COLOR_PER_VERTEX:
				drawBTMesh_->usePerVertexColors();
				break;
			case DrawModes::COLOR_PER_FACE:
				drawBTMesh_->usePerFaceColors();
				break;
			default:
			{
				drawBTMesh_->disableColors();
				ro.shaderDesc.vertexColors = false;
			} break;
		}

		// only the polygon primitives can set the normal source
		if (props->primitive() == DrawModes::PRIMITIVE_POLYGON) {
			switch (props->normalSource()) {
				case DrawModes::NORMAL_PER_VERTEX:
					drawBTMesh_->usePerVertexNormals();
					break;
				case DrawModes::NORMAL_PER_HALFEDGE:
					drawBTMesh_->usePerHalfedgeNormals();
					break;
				default: break;
			}

			if (props->flatShaded())
				drawBTMesh_->setFlatShading();
			else
				drawBTMesh_->setSmoothShading();
		}

		ro.shaderDesc.addTextureType(GL_TEXTURE_2D, false, 0);

		switch (props->texcoordSource()) {
			case DrawModes::TEXCOORD_PER_VERTEX:
				drawBTMesh_->usePerVertexTexcoords();
				break;
			case DrawModes::TEXCOORD_PER_HALFEDGE:
				drawBTMesh_->usePerHalfedgeTexcoords();
				break;
			default:
			{
				ro.shaderDesc.clearTextures();
			} break;
		}

		///////////////////////////////////////////////////////////////////////
		// Prepare renderobject
		///////////////////////////////////////////////////////////////////////

		// enable / disable lighting
		ro.shaderDesc.numLights = props->lighting() ? 0 : -1;

		// Enable/Disable twoSided Lighting
		ro.shaderDesc.twoSidedLighting = _state.twosided_lighting();

		// TODO: better handling of attribute sources in shader gen
		switch (props->lightStage()) {
			case DrawModes::LIGHTSTAGE_SMOOTH:
				ro.shaderDesc.shadeMode = SG_SHADE_GOURAUD;
				break;
			case DrawModes::LIGHTSTAGE_PHONG:
				ro.shaderDesc.shadeMode = SG_SHADE_PHONG;
				break;
			case DrawModes::LIGHTSTAGE_UNLIT:
				ro.shaderDesc.shadeMode = SG_SHADE_UNLIT;
				break;
		}

		if (props->flatShaded())
			ro.shaderDesc.shadeMode = SG_SHADE_FLAT;

		if (props->normalSource() == DrawModes::NORMAL_PER_FACE)
			ro.shaderDesc.vertexNormalInterpolator = "flat";
		else
			ro.shaderDesc.vertexNormalInterpolator.clear();

		///////////////////////////////////////////////////////////////////////
		// handle 'special' primitives (wireframe, hiddenline, primitives in sysmem buffers)..
		///////////////////////////////////////////////////////////////////////

		if (props->primitive() == DrawModes::PRIMITIVE_WIREFRAME) {
			ro.debugName = "BTMeshNode.Wireframe";

			ro.shaderDesc.shadeMode = SG_SHADE_UNLIT;
			drawBTMesh_->disableColors();

			// use specular color for lines
			if (_drawMode.isAtomic())
				ro.emissive = ro.specular;
			else
				ro.emissive = OpenMesh::color_cast<ACG::Vec3f>(_state.overlay_color());

			// allow wireframe + solid mode
			ro.depthFunc = GL_LEQUAL;
			ro.priority = -1; // render before polygon

			ro.setupLineRendering(_state.line_width(), Vec2f((float)_state.viewport_width(), (float)_state.viewport_height()));

			applyRenderObjectSettings(props->primitive(), &ro);
			add_line_RenderObjects(_renderer, &ro); // TODO
		}

		if (props->primitive() == DrawModes::PRIMITIVE_HIDDENLINE) {
			ro.shaderDesc.shadeMode = SG_SHADE_UNLIT;
			drawBTMesh_->disableColors();

			// use specular color for lines
			if (_drawMode.isAtomic())
				ro.emissive = ro.specular;
			else
				ro.emissive = OpenMesh::color_cast<ACG::Vec3f>(_state.overlay_color());

			// eventually prepare depthbuffer first
			int polyLayer = _drawMode.getLayerIndexByPrimitive(DrawModes::PRIMITIVE_POLYGON);
			if ((polyLayer > int(i) || polyLayer < 0) && (bezierTriangleMesh_.n_faces() != 0)) {
				ro.priority = 0;

				applyRenderObjectSettings(DrawModes::PRIMITIVE_POLYGON, &ro);

				// disable color write
				ro.glColorMask(0, 0, 0, 0);

				ro.debugName = "BTMeshNode.HiddenLine.faces";
				// add_face_RenderObjects(_renderer, &ro); // TODO
			}


			// draw lines after depth image
			ro.priority = 1;
			ro.glColorMask(1, 1, 1, 1);
			ro.depthFunc = GL_LEQUAL;

			ro.setupLineRendering(_state.line_width(), Vec2f((float)_state.viewport_width(), (float)_state.viewport_height()));

			applyRenderObjectSettings(DrawModes::PRIMITIVE_HIDDENLINE, &ro);

			ro.debugName = "BTMeshNode.HiddenLine.lines";
			//add_line_RenderObjects(_renderer, &ro); // TODO
		}

		if (props->colored() && props->primitive() == DrawModes::PRIMITIVE_EDGE) {
			ro.shaderDesc.shadeMode = SG_SHADE_UNLIT;
			ro.shaderDesc.vertexColors = true;

			// note: colored edges are in sysmem, so they are directly bound to the VertexDeclaration
			drawBTMesh_->updateEdgeHalfedgeVertexDeclarations();
			ro.vertexDecl = drawBTMesh_->getEdgeColoredVertexDeclaration();
			ro.glDrawArrays(GL_LINES, 0, int(bezierTriangleMesh_.n_edges() * 2));

			// use specular color for lines
			ro.emissive = ro.specular;

			// line thickness
			ro.setupLineRendering(_state.line_width(), Vec2f((float)_state.viewport_width(), (float)_state.viewport_height()));

			applyRenderObjectSettings(props->primitive(), &ro);
			ro.debugName = "BTMeshNode.Edges";
			_renderer->addRenderObject(&ro);

			// skip other edge primitives for this drawmode layer
			continue;
		}

		if (props->primitive() == DrawModes::PRIMITIVE_HALFEDGE) {
			ro.shaderDesc.shadeMode = SG_SHADE_UNLIT;

			// buffers in system memory
			drawBTMesh_->updateEdgeHalfedgeVertexDeclarations();
			halfedgeDecl.clear();
			halfedgeDecl.addElement(GL_FLOAT, 3, VERTEX_USAGE_POSITION, (void *)0);

			ro.vertexDecl = &halfedgeDecl;
			// use specular color for lines
			ro.emissive = ro.specular;
			ro.vertexBuffer = drawBTMesh_->getHEVBO();
			ro.indexBuffer = 0;
			ro.glDrawArrays(GL_LINES, 0, int(bezierTriangleMesh_.n_halfedges() * 2));

			ro.debugName = "BTMeshNode.HalfEdges";
			_renderer->addRenderObject(&ro);
		}

		///////////////////////////////////////////////////////////////////////
		// take care of all the other primitives
		///////////////////////////////////////////////////////////////////////

		ro.depthRange = Vec2f(0.01f, 1.0f);

		switch (props->primitive()) {
			case DrawModes::PRIMITIVE_POINT:
			{
				if (ro.shaderDesc.shadeMode == SG_SHADE_UNLIT) {
					// use specular color for points
					if (_drawMode.isAtomic())
						ro.emissive = ro.specular;
					else
						ro.emissive = OpenMesh::color_cast<ACG::Vec3f>(_state.overlay_color());
				}

				// use shaders to simulate point size
				ro.setupPointRendering(_mat->pointSize(), Vec2f((float)_state.viewport_width(), (float)_state.viewport_height()));

				applyRenderObjectSettings(props->primitive(), &ro);
				ro.debugName = "BTMeshNode.Points";
				//add_point_RenderObjects(_renderer, &ro); // TODO
			} break;
			case DrawModes::PRIMITIVE_EDGE:
			{
				// use specular color for lines
				ro.emissive = ro.specular;

				// use shaders to simulate line width
				ro.setupLineRendering(_state.line_width(), Vec2f((float)_state.viewport_width(), (float)_state.viewport_height()));

				applyRenderObjectSettings(props->primitive(), &ro);
				ro.debugName = "BTMeshNode.Edges";
				//add_line_RenderObjects(_renderer, &ro); // TODO
			} break;
			case DrawModes::PRIMITIVE_POLYGON:
			{
				applyRenderObjectSettings(props->primitive(), &ro);

				///////////////////////////////////////////////////////////////
				// Raytracing
				///////////////////////////////////////////////////////////////
				if (renderOption == betri::TESSELLATION_TYPE::RAYTRACING && !showBVolume) {

					// TODO this is a doublication
					if (!controlPointTex_.is_valid())
						updateTexBuffers();

					ro.shaderDesc.vertexTemplateFile = "BezierTriangle/vertex.glsl";
					ro.shaderDesc.fragmentTemplateFile = "BezierTriangle/fragment.glsl";

					//std::cerr << _state.eye() << std::endl;
					//std::cerr << _renderer->camPosWS_ << std::endl;
					//std::cerr << _renderer->viewMatrix_(0, 3) << " " << _renderer->viewMatrix_(1, 3) << " " << _renderer->viewMatrix_(2, 3) << " " << _renderer->viewMatrix_(3, 3) << std::endl;

					//ro.setUniform("campos", _renderer->camPosWS_);
					//ro.setUniform("viewMatrix", _renderer->viewMatrix_);
					ro.setUniform("campos", ACG::Vec3f(_state.eye()));

					// vertex shader uniforms
					//ro.setUniform("cameraPos", );

					// fragment shader uniforms
					static float iteration = 0.0f;
					iteration += 0.04f;
					ro.setUniform("lig", ACG::Vec3f(3.0 * cos(iteration), 3.0 * sin(iteration), 0.0));

					ro.setUniform("btriangles", int(1));
					ro.addTexture(RenderObject::Texture(controlPointTex_.id(), GL_TEXTURE_2D), 1, false);
				}

#ifdef GL_ARB_tessellation_shader
				bool tessellationMode = ACG::openGLVersion(4, 0) && Texture::supportsTextureBuffer(); // TODO

				///////////////////////////////////////////////////////////////
				// Tessellation
				///////////////////////////////////////////////////////////////
				if (tessellationMode && renderOption == betri::TESSELLATION_TYPE::GPU) {
					if (!controlPointTex_.is_valid())
						updateTexBuffers();

					ro.shaderDesc.tessControlTemplateFile = "BezierTriangle/tesscontrol_lod.glsl";
					ro.shaderDesc.tessEvaluationTemplateFile = "BezierTriangle/tesseval_lod.glsl";

					// Tesselation Control Shader Uniforms
					ro.setUniform("tessAmount", betri::mersennePrime(ITERATIONS) + 1);
					ro.setUniform("campos", ACG::Vec3f(_state.eye()));

					// Fragment Shader Uniforms
					ro.setUniform("controlPointTex", int(1));

					ro.addTexture(RenderObject::Texture(controlPointTex_.id(), GL_TEXTURE_2D), 1, false);
					//roPrimitives = GL_PATCHES;

					ro.patchVertices = 3;

					drawBTMesh_->addPatchRenderObjects(_renderer, &ro, textureMap_, false);

				} else {
#endif

					if (!ro.shaderDesc.vertexTemplateFile.isEmpty())
						drawBTMesh_->scanVertexShaderForInput(ro.shaderDesc.vertexTemplateFile.toStdString());

					bool useNonIndexed = (props->colorSource() == DrawModes::COLOR_PER_FACE) &&
						(props->lightStage() == DrawModes::LIGHTSTAGE_SMOOTH) && !props->flatShaded();
					if (!useNonIndexed && props->colorSource() == DrawModes::COLOR_PER_FACE)
						ro.shaderDesc.vertexColorsInterpolator = "flat";

					ro.debugName = "BTMeshNode.Faces";
					add_face_RenderObjects(_renderer, &ro, useNonIndexed); // TODO

					ro.shaderDesc.vertexColorsInterpolator.clear();

#ifdef GL_ARB_tessellation_shader
				}
#endif

			} break;
			default: break;
		}
#endif
#ifdef old

		///////////////////////////////////////////////////////////////////////
		//
		///////////////////////////////////////////////////////////////////////
		// generated texcoords for environment mapping should be computed in fragment shader,
		// because normals aren't available in the vertex shader
		ro.shaderDesc.texGenPerFragment = true;

		if (props->textured() && arb_texture_idx_)
			ro.addTexture(ACG::RenderObject::Texture(arb_texture_idx_), 0);

		if (props->primitive() == DrawModes::PRIMITIVE_POLYGON ||
			props->primitive() == DrawModes::PRIMITIVE_WIREFRAME)
		{
			int renderOption = betri::option(betri::BezierOption::TESSELLATION_TYPE);

			updateSurfaceMesh(renderOption);

			ro.vertexBuffer = surfaceVBO_.id();
			ro.indexBuffer = surfaceIBO_.id();
			ro.vertexDecl = &surfaceDecl_;

			if (props->primitive() == DrawModes::PRIMITIVE_WIREFRAME)
				ro.fillMode = GL_LINE;
			else
				ro.fillMode = GL_FILL;

			GLenum roPrimitives = GL_TRIANGLES;

			if (renderOption == betri::TESSELLATION_TYPE::RAYTRACING) {

				// TODO this is a doublication
				if (!controlPointTex_.is_valid())
					updateTexBuffers();

				///////////////////////
				// Additional States //
				///////////////////////
				ro.shaderDesc.shadeMode = SG_SHADE_UNLIT;

				////////////////
				// Set Shader //
				////////////////
				ro.shaderDesc.vertexTemplateFile = "BezierTriangle/vertex.glsl";
				ro.shaderDesc.fragmentTemplateFile = "BezierTriangle/fragment.glsl";

				/////////////
				// Defines //
				/////////////
				QString shaderMacro;
				shaderMacro.sprintf("#define NEWTON_IT_COUNT %i", betri::option(betri::BezierOption::NEWTON_IT_COUNT));
				ro.shaderDesc.macros.push_back(shaderMacro);

				if (betri::option(betri::BezierOption::VISUALISATION_MODE) == betri::VIS_MODE::PHONGCOLOR)
					shaderMacro.sprintf("#define SG_OUTPUT_PHONGCOLOR");
				else if (betri::option(betri::BezierOption::VISUALISATION_MODE) == betri::VIS_MODE::COLOR)
					shaderMacro.sprintf("#define SG_OUTPUT_COLOR");
				else if (betri::option(betri::BezierOption::VISUALISATION_MODE) == betri::VIS_MODE::NORMAL)
					shaderMacro.sprintf("#define SG_OUTPUT_NORMAL");
				else if (betri::option(betri::BezierOption::VISUALISATION_MODE) == betri::VIS_MODE::DEPTH)
					shaderMacro.sprintf("#define SG_OUTPUT_DEPTH");
				else
					shaderMacro.sprintf("#define SG_OUTPUT_CURVATURE");
				ro.shaderDesc.macros.push_back(shaderMacro);

				shaderMacro.sprintf("#define CPSUM %i", cpSum());
				ro.shaderDesc.macros.push_back(shaderMacro);

				shaderMacro.sprintf("#define GRAD %i", grad());
				ro.shaderDesc.macros.push_back(shaderMacro);

				int showBVolume = betri::option(betri::BezierOption::SHOW_BOUNDING_VOLUME);
				if (showBVolume) {
					shaderMacro.sprintf("#define SHOWBVOLUME");
					ro.shaderDesc.macros.push_back(shaderMacro);
				}

				//////////////
				// Uniforms //
				//////////////
				//std::cerr << _state.eye() << std::endl;
				//std::cerr << _renderer->camPosWS_ << std::endl;
				//std::cerr << _renderer->viewMatrix_(0, 3) << " " << _renderer->viewMatrix_(1, 3) << " " << _renderer->viewMatrix_(2, 3) << " " << _renderer->viewMatrix_(3, 3) << std::endl;

				// vertex shader uniforms
				//ro.setUniform("campos", _renderer->camPosWS_);
				//ro.setUniform("viewMatrix", _renderer->viewMatrix_);
				//ro.setUniform("cameraPos", );
				ro.setUniform("campos", ACG::Vec3f(_state.eye()));

				// fragment shader uniforms
				ro.setUniform("b_error", (1.0f / betri::option(betri::BezierOption::B_ERROR)));
				ro.setUniform("d_error", (1.0f / betri::option(betri::BezierOption::D_ERROR)));

				ro.setUniform("btriangles", int(1));
				ro.addTexture(RenderObject::Texture(controlPointTex_.id(), GL_TEXTURE_2D), 1, false);
			}

#ifdef GL_ARB_tessellation_shader
			bool tessellationMode = ACG::openGLVersion(4, 0) && Texture::supportsTextureBuffer(); // TODO

			if (tessellationMode && renderOption == betri::TESSELLATION_TYPE::GPU)
			{
				// dynamic lod tessellation and spline evaluation on gpu

				if (!controlPointTex_.is_valid())
					updateTexBuffers();

				////////////////
				// Set Shader //
				////////////////
				ro.shaderDesc.tessControlTemplateFile = "BezierTriangle/tesscontrol_lod.glsl";
				ro.shaderDesc.tessEvaluationTemplateFile = "BezierTriangle/tesseval_lod.glsl";

				/////////////
				// Defines //
				/////////////
				// TODO why is it possible to submit the degree
				// as an define but not as an uniform?
				// Liegt das an der for-schleife?
				QString shaderMacro;
				shaderMacro.sprintf("#define DEGREE %i", grad());
				ro.shaderDesc.macros.push_back(shaderMacro);

				if (betri::option(betri::BezierOption::TESSELLATION_ADAPTIVE) == 1) {
					shaderMacro.sprintf("#define TESS_DISTANCE");
					ro.shaderDesc.macros.push_back(shaderMacro);
				} else if (betri::option(betri::BezierOption::TESSELLATION_ADAPTIVE) == 2) {
					shaderMacro.sprintf("#define TESS_FLATNESS");
					ro.shaderDesc.macros.push_back(shaderMacro);
				} else {
					shaderMacro.sprintf("#define TESS_CONST");
					ro.shaderDesc.macros.push_back(shaderMacro);
				}

				//////////////
				// Uniforms //
				//////////////
				ro.setUniform("tessAmount", betri::mersennePrime(ITERATIONS) + 1);
				ro.setUniform("campos", ACG::Vec3f(_state.eye()));
				ro.setUniform("controlPointTex", int(1));

				// Textures ---------------------------------------------------
				ro.addTexture(RenderObject::Texture(controlPointTex_.id(), GL_TEXTURE_2D), 1, false);
				//ro.addTexture(RenderObject::Texture(knotTexBufferU_.id(), GL_TEXTURE_BUFFER), 2, false);
				//ro.addTexture(RenderObject::Texture(knotTexBufferV_.id(), GL_TEXTURE_BUFFER), 3, false);

				roPrimitives = GL_PATCHES;
			}

			if (tessellationMode)
				ro.patchVertices = 3;
#endif

			ro.glDrawElements(roPrimitives, surfaceIndexCount_, GL_UNSIGNED_INT, 0);

			_renderer->addRenderObject(&ro);
		}
#endif
	}


	///////////////////////////////////////////////////////////////////////////
	// draw the control net (includes selection on the net)
	///////////////////////////////////////////////////////////////////////////
	// TODO
	if (render_control_net_)
	{
		// update if necessary
		updateControlNetMesh();
		updateControlNetMeshSel();

		// setup base renderobject for unlit point and line rendering
		RenderObject ro;
		ro.initFromState(&_state);
		ro.depthTest = true;
		ro.shaderDesc.shadeMode = SG_SHADE_UNLIT;

		ro.vertexBuffer = controlNetVBO_.id();
		ro.vertexDecl = &controlNetDecl_;

		Vec2f screenSize = Vec2f(_state.viewport_width(), _state.viewport_height());

		// selected control points
		if (controlNetSelIndices_)
		{
			ro.name = "BezierTriangleMesh__ControlPointSel";

			ro.setupPointRendering(10.0f, screenSize);

			Vec4f selColor = generateHighlightColor(controlnet_color_);
			ro.emissive = Vec3f(selColor[0], selColor[1], selColor[2]);

			ro.indexBuffer = controlNetSelIBO_.id();

			ro.glDrawElements(GL_POINTS, controlNetSelIndices_, GL_UNSIGNED_INT, 0);

			_renderer->addRenderObject(&ro);
		}

		// all control points
		{
			ro.name = "BezierTriangleMesh_ControlPoint";
			ro.setupPointRendering(_state.point_size() + 4.0f, screenSize);

			ro.emissive = Vec3f(controlpoints_color_[0], controlpoints_color_[1], controlpoints_color_[2]);

			GLsizei numControlPoints = cpSum() * bezierTriangleMesh_.n_faces();
			ro.glDrawArrays(GL_POINTS, 0, numControlPoints);

			_renderer->addRenderObject(&ro);
		}

		ro.resetPointRendering();

		// all line segments
		{
			ro.name = "BezierTriangleMesh_ControlNetLines";
			ro.setupLineRendering(_state.line_width() + 2.0f, screenSize);

			ro.indexBuffer = controlNetLineIBO_.id();

			ro.glDrawElements(GL_LINES, controlNetLineIndices_, GL_UNSIGNED_INT, 0);

			_renderer->addRenderObject(&ro);
		}
	}

	double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

	//std::cerr << "duration: " << duration << " FPS " << (1 / duration) << '\n';
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::add_face_RenderObjects(
	IRenderer* _renderer, const RenderObject* _baseObj, bool _nonindexed
)
{
	drawBTMesh_->addTriRenderObjects(_renderer, _baseObj, textureMap_, _nonindexed);
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::add_line_RenderObjects(
	IRenderer* _renderer, const RenderObject* _baseObj
)
{
//	if ((enabled_arrays_ & PER_EDGE_COLOR_ARRAY) && (enabled_arrays_ & PER_EDGE_VERTEX_ARRAY)) {
		// colored edges still slow
//		glDrawArrays(GL_LINES, 0, int(drawBTMesh_.n_edges() * 2));
//	} else
		drawBTMesh_->addLineRenderObjects(_renderer, _baseObj);
}

///////////////////////////////////////////////////////////////////////////////
// Draw Forward Functions
///////////////////////////////////////////////////////////////////////////////

/**
 * This function is not called if the getRenderObjects() is used
 */
template <class MeshT>
void BezierTriangleMeshNode<MeshT>::draw(
	GLState& _state, const DrawModes::DrawMode& _drawMode
)
{
	// only render mesh if it is ready
	if (!bezierTriangleMesh_.isRenderable())
		return;

	// TODO
	if (controlPointsChangedC_) {
		//setControlPointsCircular();
		setControlPointsColumnwise();
		controlPointsChangedC_ = false;
		controlPointsChangedR_ = true;
	}

	GLenum prev_depth = _state.depthFunc();

	glPushAttrib(GL_ENABLE_BIT);

	// check if textures are still valid
	if (bspline_selection_draw_mode_ == CONTROLPOINT
		&& controlPointSelectionTexture_valid_ == false)
		updateControlPointSelectionTexture(_state);
	if (bspline_selection_draw_mode_ == KNOTVECTOR
		&& knotVectorSelectionTexture_valid_ == false)
		updateKnotVectorSelectionTexture(_state);


	if (_drawMode & DrawModes::POINTS)
	{
		ACG::GLState::disable(GL_LIGHTING);
		ACG::GLState::shadeModel(GL_FLAT);
		render(_state, false);
	}

	if (_drawMode & DrawModes::WIREFRAME)
	{
		glPushAttrib(GL_ENABLE_BIT);

		ACG::GLState::disable(GL_CULL_FACE);
		ACG::GLState::disable(GL_LIGHTING);
		ACG::GLState::shadeModel(GL_FLAT);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		render(_state, false);

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glPopAttrib();
	}


	if (_drawMode & DrawModes::HIDDENLINE)
	{
		Vec4f  clear_color = _state.clear_color();
		Vec4f  base_color = _state.base_color();
		clear_color[3] = 1.0;

		ACG::GLState::enable(GL_DEPTH_TEST);
		ACG::GLState::disable(GL_LIGHTING);
		ACG::GLState::shadeModel(GL_FLAT);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		_state.set_base_color(clear_color);

		ACG::GLState::depthRange(0.01, 1.0);

		render(_state, true);

		ACG::GLState::depthRange(0.0, 1.0);

		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		ACG::GLState::depthFunc(GL_LEQUAL);
		_state.set_base_color(base_color);

		render(_state, false);

		ACG::GLState::depthFunc(prev_depth);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}


	if ((_drawMode & DrawModes::SOLID_FLAT_SHADED))
	{
		ACG::GLState::enable(GL_LIGHTING);
		ACG::GLState::shadeModel(GL_FLAT);

		ACG::GLState::enable(GL_AUTO_NORMAL);
		ACG::GLState::enable(GL_NORMALIZE);

		ACG::GLState::depthRange(0.01, 1.0);

		render(_state, true);

		ACG::GLState::depthRange(0.0, 1.0);
	}

	if ((_drawMode & DrawModes::SOLID_SMOOTH_SHADED))
	{
		ACG::GLState::enable(GL_AUTO_NORMAL);
		ACG::GLState::enable(GL_NORMALIZE);

		ACG::GLState::enable(GL_LIGHTING);
		ACG::GLState::shadeModel(GL_SMOOTH);
		ACG::GLState::depthRange(0.01, 1.0);

		render(_state, true);

		ACG::GLState::depthRange(0.0, 1.0);
	}

	if ((_drawMode & DrawModes::SOLID_PHONG_SHADED))
	{
		ACG::GLState::enable(GL_AUTO_NORMAL);
		ACG::GLState::enable(GL_NORMALIZE);

		ACG::GLState::enable(GL_LIGHTING);
		ACG::GLState::shadeModel(GL_SMOOTH);
		ACG::GLState::depthRange(0.01, 1.0);

		render(_state, true);

		ACG::GLState::depthRange(0.0, 1.0);
	}


	// If in shader mode, just draw, as the shader has to be set by a shadernode above this node
	if ((_drawMode & DrawModes::SOLID_SHADER)) {
		ACG::GLState::enable(GL_AUTO_NORMAL);
		ACG::GLState::enable(GL_NORMALIZE);

		ACG::GLState::enable(GL_LIGHTING);
		ACG::GLState::shadeModel(GL_SMOOTH);
		ACG::GLState::depthRange(0.01, 1.0);

		//     draw_faces(PER_VERTEX);
		render(_state, true);

		ACG::GLState::depthRange(0.0, 1.0);
	}


	if ((_drawMode & DrawModes::SOLID_TEXTURED) || (_drawMode & DrawModes::SOLID_ENV_MAPPED)) {
		ACG::GLState::enable(GL_AUTO_NORMAL);
		ACG::GLState::enable(GL_NORMALIZE);
		//     ACG::GLState::enable (GL_BLEND);
		//     ACG::GLState::blendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		ACG::GLState::enable(GL_LIGHTING);
		ACG::GLState::shadeModel(GL_SMOOTH);
		ACG::GLState::depthRange(0.01, 1.0);

		arb_texture_used_ = true;
		drawTexturedSurface(_state, arb_texture_idx_);
		arb_texture_used_ = false;

		ACG::GLState::depthRange(0.0, 1.0);
		//    ACG::GLState::disable(GL_BLEND);
	}

	/*
	// BIG TODO here should be happening smth else, but i dont know how to color the faces
	if ((_drawMode & DrawModes::SOLID_FACES_COLORED)) {
		ACG::GLState::enable(GL_COLOR_MATERIAL);

		ACG::GLState::enable(GL_AUTO_NORMAL);
		ACG::GLState::enable(GL_NORMALIZE);

		ACG::GLState::enable(GL_LIGHTING);
		ACG::GLState::shadeModel(GL_SMOOTH);
		ACG::GLState::depthRange(0.01, 1.0);

		render(_state, true);

		ACG::GLState::depthRange(0.0, 1.0);
	}*/

	glPopAttrib();
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::render(GLState& _state, bool _fill)
{
	// draw the control net (includes selection on the net)

	if (render_control_net_) // TODO diese variable muss setzbar sein
	{
		if (bspline_draw_mode_ == NORMAL)
			drawControlNet(_state);
		else if (bspline_draw_mode_ == FANCY)
			drawFancyControlNet(_state);
	}


	// draw the spline curve itself, depending on the type of visualization
	if (render_bspline_surface_)
	{
		if (bspline_selection_draw_mode_ == NONE)
			drawSurface(_state, _fill);
		// TODO wo kommt die textur her und wie wird eingestellt das die gesampled wird?
		else if (bspline_selection_draw_mode_ == CONTROLPOINT)
			drawTexturedSurface(_state, cp_selection_texture_idx_);
		//else if (bspline_selection_draw_mode_ == KNOTVECTOR)
		//	drawTexturedSurface(_state, knot_selection_texture_idx_);
	}
}

//-----------------------------------------------------------------------------

/**
 * This method is evaluated once everytime the user moves the Camera.
 * This works since there are no dynamic elements (lights) that whould change the image.
 */
template <class MeshT>
void BezierTriangleMeshNode<MeshT>::drawSurface(GLState& _state, bool _fill)
{
	int renderOption = betri::option(betri::BezierOption::TESSELLATION_TYPE);
	updateSurfaceMesh(renderOption);

	surfaceVBO_.bind();
	surfaceIBO_.bind();

	surfaceDecl_.activateFixedFunction();

	// draw
	glDrawElements(GL_TRIANGLES, surfaceIndexCount_, GL_UNSIGNED_INT, 0);

	surfaceDecl_.deactivateFixedFunction();

	surfaceIBO_.unbind();
	surfaceVBO_.unbind();
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::drawTexturedSurface(
	GLState& _state, GLuint _texture_idx
)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//  ACG::GLState::enable( GL_COLOR_MATERIAL );
	//  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

	ACG::GLState::enable(GL_TEXTURE_2D);

	// blend colors (otherwise lighting does not affect the texture)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	// avoid aliasing at patch boundaries
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	// repeat if arbitrary texture mode
	if (arb_texture_used_) {
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	}

	// GL_MODULATE to include lighting effects
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	ACG::GLState::bindTexture(GL_TEXTURE_2D, _texture_idx);

	drawSurface(_state);

	ACG::GLState::bindTexture(GL_TEXTURE_2D, 0);
	ACG::GLState::disable(GL_TEXTURE_2D);
	//  ACG::GLState::disable( GL_COLOR_MATERIAL );
	glPopAttrib();
}

///////////////////////////////////////////////////////////////////////////////
// Controllnet Functions
///////////////////////////////////////////////////////////////////////////////

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::drawControlNet(GLState& _state)
{
	// remember old color
	Vec4f base_color_old = _state.base_color();

	// draw control net
	//   glPushAttrib(GL_ENABLE_BIT);
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	ACG::GLState::disable(GL_CULL_FACE);
	ACG::GLState::disable(GL_LIGHTING);
	ACG::GLState::shadeModel(GL_FLAT);


	// update controlnet buffers
	updateControlNetMesh();
	updateControlNetMeshSel();

	// bind vbo containing all control points
	controlNetVBO_.bind();
	controlNetDecl_.activateFixedFunction();

	// draw points

	// draw selection
	if (controlNetSelIndices_) // TODO
	{
		glColor(generateHighlightColor(controlnet_color_));
		glPointSize(10);

		// selected points are in index buffer
		controlNetSelIBO_.bind();
		glDrawElements(GL_POINTS, controlNetSelIndices_, GL_UNSIGNED_INT, 0);
	}


	// draw all points
	glColor(controlpoints_color_);

	float point_size_old = _state.point_size();
	glPointSize(point_size_old + 4);

	GLsizei numControlPoints = cpSum() * bezierTriangleMesh_.n_faces();
	glDrawArrays(GL_POINTS, 0, numControlPoints);

	glPointSize((int)point_size_old);
	/*

	// draw line segments

	// draw selection
	if( bezierTriangleMesh_.edge_selections_available())
	{
	  // save old values
	  Vec4f base_color_old = _state.base_color();
	  float line_width_old = _state.line_width();

	  glColor(controlnet_highlight_color_);
	  glLineWidth(2*line_width_old);

	  glBegin(GL_LINES);
	  // draw bspline control net
	  int num_edges_m = (int)(bezierTriangleMesh_.n_control_points_m()) - 1;
	  int num_edges_n = (int)(bezierTriangleMesh_.n_control_points_n()) - 1;
	  for (int i = 0; i < num_edges_m; ++i) // #edges
	  {
		for (int j = 0; j < num_edges_n; ++j) // #edges
		{
		  if( bezierTriangleMesh_.edge_selection(i, j))
		  {
			glVertex(bezierTriangleMesh_(i,j));
			glVertex(bezierTriangleMesh_(i+1, j));
		  }
		}
	  }
	  glEnd();

	  glLineWidth(line_width_old);
	  glColor( base_color_old );
	}
	*/
	// draw all line segments

	glColor(controlnet_color_);

	float line_width_old = _state.line_width();
	glLineWidth(line_width_old + 2.0);

	controlNetLineIBO_.bind();
	glDrawElements(GL_LINES, controlNetLineIndices_, GL_UNSIGNED_INT, 0);

	// restore gl states
	controlNetDecl_.deactivateFixedFunction();
	controlNetLineIBO_.unbind();
	controlNetVBO_.unbind();

	glColor(base_color_old);
	glLineWidth(line_width_old);

	glPopAttrib();

}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::drawFancyControlNet(GLState& _state)
{

	// remember old color
	Vec4f base_color_old = _state.base_color();

	// draw control net
  //   glPushAttrib(GL_ENABLE_BIT);
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	ACG::GLState::disable(GL_CULL_FACE);

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	ACG::GLState::enable(GL_COLOR_MATERIAL);
	ACG::GLState::enable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	ACG::GLState::shadeModel(GL_SMOOTH);


	// draw points
	double sphereRadius = _state.point_size() * 0.05;
	/*
	// draw selection
	if (bezierTriangleMesh_.controlpoint_selections_available())
	{
		// save old values
		float point_size_old = _state.point_size();

		// save old values
	//     glColor(controlnet_highlight_color_);
		glColor(generateHighlightColor(controlnet_color_));

		// draw control polygon
		for (unsigned int i = 0; i < bezierTriangleMesh_.n_control_points_m(); ++i)
		{
			for (unsigned int j = 0; j < bezierTriangleMesh_.n_control_points_n(); ++j)
			{
				if (bezierTriangleMesh_.controlpoint_selection(i, j))
					draw_sphere(bezierTriangleMesh_(i, j), sphereRadius, _state, fancySphere_);
			}
		}

		glPointSize(point_size_old);
	}
	*/

	// draw all points
	glColor(controlpoints_color_);

	const int controlPointsPerFace = cpSum();

	for (auto &face : bezierTriangleMesh_.faces()) {
		auto faceControlP = bezierTriangleMesh_.data(face);
		Point cp;
		for (int i = 0; i < controlPointsPerFace; i++) {
			cp = faceControlP.controlPoint(i);
			draw_sphere(cp, sphereRadius, _state, fancySphere_);
		}
	}

	// draw line segments

	double cylinderRadius = _state.line_width() * 0.05;

	glColor(controlnet_color_);

	for (FaceHandle face : bezierTriangleMesh_.faces()) {
		auto faceControlP = bezierTriangleMesh_.data(face);
		Point cp;

		int pos1 = 0;
		int pos2 = 1;
		int pos3 = cpCount() + 2;
		int border = cpCount() + 2;
		int boderAdd = cpCount() + 2 - 1;

		const int controlPointsPerFace = cpSum();
		for (; pos3 < controlPointsPerFace; ) {
			Point p1 = faceControlP.controlPoint(pos1);
			Point p2 = faceControlP.controlPoint(pos2);
			Point p3 = faceControlP.controlPoint(pos3);
			// TODO warum hier p2-p1?
			draw_cylinder(p1, p2 - p1, cylinderRadius, _state);
			draw_cylinder(p2, p3 - p2, cylinderRadius, _state);
			draw_cylinder(p3, p1 - p3, cylinderRadius, _state);

			if (pos2 + 1 == border) {
				border += boderAdd--;
				pos1++;
				pos2++;
			}

			pos1++;
			pos2++;
			pos3++;

		}
	}

	/* OLD
	// draw bezierTriangle control net
	for (unsigned int i = 0; i < bezierTriangleMesh_.n_control_points_m(); ++i)
	{
		for (int j = 0; j < (int)bezierTriangleMesh_.n_control_points_n() - 1; ++j)
		{
			Vec3d p = bezierTriangleMesh_(i, j);
			Vec3d p_next = bezierTriangleMesh_(i, j + 1);
			draw_cylinder(p, p_next - p, cylinderRadius, _state);
		}
	}
	*/
	// reset old color
	glColor(base_color_old);

	glPopAttrib();

}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::draw_sphere(
	const Point& _p0, double _r, GLState& _state, GLSphere* _sphere
)
{
	// draw 3d sphere
	_state.push_modelview_matrix();
	_state.translate(_p0[0], _p0[1], _p0[2]);

	_sphere->draw(_state, _r);

	_state.pop_modelview_matrix();
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::draw_cylinder(
	const Point& _p0, const Point& _axis, double _r, GLState& _state
)
{
	_state.push_modelview_matrix();
	_state.translate(_p0[0], _p0[1], _p0[2]);

	Point direction = _axis;
	Point z_axis(0, 0, 1);
	Point rot_normal;
	double rot_angle;

	direction.normalize();
	rot_angle = acos((z_axis | direction)) * 180 / M_PI;
	rot_normal = ((z_axis % direction).normalize());

	if (fabs(rot_angle) > 0.0001 && fabs(180 - rot_angle) > 0.0001)
		_state.rotate(rot_angle, rot_normal[0], rot_normal[1], rot_normal[2]);
	else
		_state.rotate(rot_angle, 1, 0, 0);

	cylinder_->setBottomRadius(_r);
	cylinder_->setTopRadius(_r);
	cylinder_->draw(_state, _axis.norm());

	_state.pop_modelview_matrix();
}

///////////////////////////////////////////////////////////////////////////////
// Update Functions
///////////////////////////////////////////////////////////////////////////////
template <class MeshT>
void BezierTriangleMeshNode<MeshT>::updateGeometry()
{
	invalidateSurfaceMesh_ = true;
	invalidateControlNetMesh_ = true;

	drawBTMesh_->invalidateSurfaceMesh_ = true;

	// TODO
	NEWVERTICES = betri::mersennePrime(ITERATIONS);
	VERTEXSUM = betri::gaussSum(NEWVERTICES + 2);
	STEPSIZE = 1.0 / (double(NEWVERTICES) + 1.0);

	drawBTMesh_->NEWVERTICES = betri::mersennePrime(ITERATIONS);
	drawBTMesh_->VERTEXSUM = betri::gaussSum(drawBTMesh_->NEWVERTICES + 2);
	drawBTMesh_->STEPSIZE = 1.0 / (double(drawBTMesh_->NEWVERTICES) + 1.0);

	updateTexBuffers(); // TODO
}

// Surface --------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::updateSurfaceDecl()
{
	// vertex layout:
	//  float3 pos
	//  float3 normal
	//  float2 texcoord
	//  + debug info (optional)

	// provide expected values of bspline evaluation steps for debugging in shader
	const bool provideDebugInfo = false;

	surfaceDecl_.clear();
	if (!surfaceDecl_.getNumElements()) {
		int renderOption = betri::option(betri::BezierOption::TESSELLATION_TYPE);

		surfaceDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_POSITION);
		// TODO the normal is not needed for raytracing but i was not able to remove it from the shader
		if (renderOption != betri::TESSELLATION_TYPE::RAYTRACING) {
			surfaceDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_NORMAL);
		}
		surfaceDecl_.addElement(GL_FLOAT, 2, VERTEX_USAGE_TEXCOORD);
		if (renderOption != betri::TESSELLATION_TYPE::RAYTRACING) {
			surfaceDecl_.addElement(GL_FLOAT, 4, VERTEX_USAGE_COLOR);
		}
		//surfaceDecl_.addElement(GL_UNSIGNED_BYTE, 4, VERTEX_USAGE_COLOR); TODO


		if (provideDebugInfo) {
			surfaceDecl_.addElement(GL_FLOAT, 2, VERTEX_USAGE_SHADER_INPUT, size_t(0), "a2v_span");
			surfaceDecl_.addElement(GL_FLOAT, 4, VERTEX_USAGE_SHADER_INPUT, size_t(0), "a2v_bvu");
			surfaceDecl_.addElement(GL_FLOAT, 4, VERTEX_USAGE_SHADER_INPUT, size_t(0), "a2v_bvv");
		}
	}
}

//-----------------------------------------------------------------------------

/**
 * This method is only evaluated if the Mesh was changed (invalidateSurfaceMesh_)
 * This is the case if updateGeometry() is called.
 */
template <class MeshT>
void BezierTriangleMeshNode<MeshT>::updateSurfaceMesh(const int meshOption) // TODO methode ueberladen, statt pointer?
{
	//std::cerr << "IN updateSurfaceMesh: " << " isColored? " << props->colored() << std::endl;
	//std::cerr << "IN updateSurfaceMesh: colorSource " << props->colorSource() << std::endl;

	// TODO this is dump
	if (!invalidateSurfaceMesh_ &&
		betri::option(betri::BezierOption::BOUNDING_VOLUME) != betri::boundingVolumeType::BoundingBillboard)
		return;

	surfaceVBO_.del();
	surfaceIBO_.del();

	updateSurfaceDecl();

	if (false) { // TODO if apply tesselation - should get a separate call
		// TODO
		// TODO should the mesh really be changed? we could simple apply the
		// changes to the vbo and dont change the Mesh itself
		// TODO Button f�r applyTesselation
		tesselateMeshCPU();
	}

	// TODO Performance verbessern indem in den vertex buffer alle vertices gepackt werden und dann
	// beim index buffer die indices direkt genutzt werden

	// TODO
	// not really sure what happens - but the renderOption should
	// decide if there is CPU or GPU tesselation (or both), the render mode needs to
	// change based on that
	// Generate a VBO from the Mesh without CPU tesselation
	if (meshOption == betri::TESSELLATION_TYPE::GPU || meshOption == betri::TESSELLATION_TYPE::NONE) {
		VBOfromMesh();
	}
	// Generate a VBO and apply CPU tesselation without changing the Mesh
	else if (meshOption == betri::TESSELLATION_TYPE::CPU) {
		VBOtesselatedFromMesh();
	} else if (meshOption == betri::TESSELLATION_TYPE::RAYTRACING) {
		VBOfromBoundingMesh();
	}
}

// ControlNet -----------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::updateControlNetMesh()
{
#ifdef RENDER_DEBUG
	std::ofstream out("03controlPMUpdate.txt", std::ios::out | std::ofstream::app);
#endif
	if (!invalidateControlNetMesh_)
		return;

	// vertex layout:
	//  float3 pos

	// TODO H�?
	if (!controlNetDecl_.getNumElements())
		controlNetDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_POSITION);

	const int controlPointsPerFace = cpSum();
	int controlPointCountSum = bezierTriangleMesh_.n_faces() * controlPointsPerFace;

	// create vertex buffer
	GLsizeiptr vboSize = controlPointCountSum * controlNetDecl_.getVertexStride(); // bytes
	std::vector<float> vboData(vboSize / 4); // float: 4 bytes

	int elementOffset = 0;
	for (auto &face : bezierTriangleMesh_.faces()) {
		// write counter

		//auto faceControlP = bezierTriangleMesh_.data(face);
		//Point pt = faceControlP.controlPoint(i); // TODO

		auto faceControlP = bezierTriangleMesh_.data(face);
		Point cp;
		for (int i = 0; i < controlPointsPerFace; i++) {
			cp = faceControlP.controlPoint(i);
			for (int m = 0; m < 3; ++m)
				vboData[elementOffset++] = cp[m];
		}
	}

	if (vboSize)
		controlNetVBO_.upload(vboSize, &vboData[0], GL_STATIC_DRAW);

	vboData.clear();

	// TODO more tests that this is correct for all cases and that the index counts are corrects (idxOffset vs numIndices)

	int bottomTriangles = betri::gaussSum(grad());
	// TODO unterschiedliche Faces k�nnen unterschiedliche kontrollpunkte haben auch wenn sie aneinanderliegen?! deswegen mehrere Linien an der grenze ?
	const int linesPerTriangle = 3;
	const int pointPerLine = 2;
	int numIndices = bottomTriangles * linesPerTriangle * pointPerLine * bezierTriangleMesh_.n_faces();
	std::vector<int> iboData(numIndices);

	int faceindex = 0, idxOffset = 0;
	for (auto &face : bezierTriangleMesh_.faces()) {

		const int controlPointsPerFace = cpSum();
		int offset = faceindex * controlPointsPerFace;
		int pos1 = offset + 0;
		int pos2 = offset + 1;
		int pos3 = offset + cpCount() + 2;
		int border = offset + cpCount() + 2;
		int boderAdd = cpCount() + 2 - 1;

		for (; pos3 < (faceindex + 1) * controlPointsPerFace; ) {
			iboData[idxOffset++] = pos1;
			iboData[idxOffset++] = pos2;
			iboData[idxOffset++] = pos2;
			iboData[idxOffset++] = pos3;
			iboData[idxOffset++] = pos3;
			iboData[idxOffset++] = pos1;

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

	if (numIndices)
		controlNetLineIBO_.upload(numIndices * 4, &iboData[0], GL_STATIC_DRAW);

	controlNetLineIndices_ = numIndices;

	invalidateControlNetMesh_ = false;
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::updateTexBuffers()
{
	/*
	const size_t knotBufSizeU = bezierTriangleMesh_.get_knots_m().size();
	const size_t knotBufSizeV = bezierTriangleMesh_.get_knots_n().size();




	if (knotBufSizeU)
	{
		std::vector<float> knotBufU(knotBufSizeU);

		for (size_t i = 0; i < knotBufSizeU; ++i)
			knotBufU[i] = float(bezierTriangleMesh_.get_knot_m(i));

		knotTexBufferU_.setBufferData(knotBufSizeU * 4, &knotBufU[0], GL_R32F);
	}

	if (knotBufSizeV)
	{
		std::vector<float> knotBufV(knotBufSizeV);

		for (size_t i = 0; i < knotBufSizeV; ++i)
			knotBufV[i] = float(bezierTriangleMesh_.get_knot_n(i));

		knotTexBufferV_.setBufferData(knotBufSizeV * 4, &knotBufV[0], GL_R32F);
	}

	*/
#ifdef GL_VERSION_3_0

	const int controlPointsPerFace = cpSum();
	const size_t controlPointBufSize = controlPointsPerFace * bezierTriangleMesh_.n_faces();

	if (controlPointBufSize) {
		std::vector<float> controlPointBuf(controlPointBufSize * 3);

		int elementOffset = 0;
		for (auto &face : bezierTriangleMesh_.faces()) {
			// write counter

			auto faceControlP = bezierTriangleMesh_.data(face);
			Point cp;
			for (int i = 0; i < controlPointsPerFace; i++) {
				cp = faceControlP.controlPoint(i);
				for (int m = 0; m < 3; ++m)
					controlPointBuf[elementOffset++] = cp[m];
			}
		}

		controlPointTex_.bind();
		controlPointTex_.parameter(GL_TEXTURE_MIN_FILTER, GL_NEAREST); // disable filtering
		controlPointTex_.parameter(GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		controlPointTex_.setData(0, GL_RGB32F, controlPointsPerFace, bezierTriangleMesh_.n_faces(), GL_RGB, GL_FLOAT, &controlPointBuf[0]);
	}

#endif
}

///////////////////////////////////////////////////////////////////////////////
// Functions for VBO creation
///////////////////////////////////////////////////////////////////////////////

template <class MeshT>
BezierTMesh::Point BezierTriangleMeshNode<MeshT>::evaluateCasteljau(
	Point at, Point cp0, Point cp1, Point cp2, Point cp3, Point cp4, Point cp5
)
{
	auto tmpPointA = cp0 * at[0] + cp1 * at[1] + cp5 * at[2];
	auto tmpPointB = cp1 * at[0] + cp2 * at[1] + cp3 * at[2];
	auto tmpPointC = cp5 * at[0] + cp3 * at[1] + cp4 * at[2];

	auto result = tmpPointA * at[0] + tmpPointB * at[1] + tmpPointC * at[2];

	return result;
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::tesselateMeshCPU()
{
	Point cp0, cp1, cp2, cp3, cp4, cp5;
	// Iterate over all faces
	for (auto &face : bezierTriangleMesh_.faces()) {
		// TODO is this nessessary?
		auto vertexHandle = bezierTriangleMesh_.fv_begin(face);
		auto vh0 = *(vertexHandle++);
		auto vh1 = *(vertexHandle++);
		auto vh2 = *(vertexHandle);

		// Delete the old face
		bezierTriangleMesh_.delete_face(face, false);

		// Get the controlpoints of this face

		// TODO read the controllpoints from the mesh data
		auto faceControlP = bezierTriangleMesh_.data(face);
		cp0 = faceControlP.controlPoint(0);
		cp1 = faceControlP.controlPoint(1);
		cp2 = faceControlP.controlPoint(2);
		cp3 = faceControlP.controlPoint(3);
		cp4 = faceControlP.controlPoint(4);
		cp5 = faceControlP.controlPoint(5);

		std::vector<BezierTMesh::VertexHandle> newHandleVector = std::vector<BezierTMesh::VertexHandle>(VERTEXSUM);
		// Iterate in two directions (u,v) which can use to determine the point at which
		// the beziertriangle should be evaluated
		int handleIt = 0;
		for (double u = 0.0; u <= 1.0; u += STEPSIZE) {
			for (double v = 0.0; u + v <= 1.0; v += STEPSIZE) {

				// Get the 3D-position
				auto toEval = betri::getBaryCoords(u, v);
				//auto toEval = Point(u, v, 1.0 - u - v);

				auto resultPoint = evaluateCasteljau(toEval, cp0, cp1, cp2, cp3, cp4, cp5);

				// Add Point
				// TODO dont add the Points that are already in there (3 starting points)
				auto newPointHandle = bezierTriangleMesh_.add_vertex(resultPoint);
				newHandleVector[handleIt++] = newPointHandle;
			}
		}

		// Example - first half of the triangles
		// 0 1 5 b=5
		// 1 2 6 b=5
		// 2 3 7 b=5
		// 3 4 8 b=5
		// pos1+2 pos2+2 pos3+1 b=5+4
		// 5 6 9 b=9
		// 6 7 10 b=9
		// 7 8 11 b=9
		// pos1+2 pos2+2 pos3+1 b=5+4+3
		// 9 10 12 b=12
		// 10 11 13 b=12
		// pos1+2 pos2+2 pos3+1 b=5+4+3+2
		// 12 13 14 b=14

		int pos1 = 0;
		int pos2 = 1;
		int pos3 = NEWVERTICES + 2;
		int border = NEWVERTICES + 2;
		int boderAdd = border-1;

		// Iterate all added Points and add pairs of three as a new face
		for (; pos3 < newHandleVector.size(); ) {
			// bottom triangle
			auto faceHandle = bezierTriangleMesh_.add_face(newHandleVector[pos1], newHandleVector[pos2], newHandleVector[pos3]);
			// Add the controllPoints to the face
			bezierTriangleMesh_.data(faceHandle).points(std::vector<Point>({ cp0, cp1, cp2, cp3, cp4, cp5 }));

			if (pos2 + 1 < border) {
				// top triangle
				faceHandle = bezierTriangleMesh_.add_face(newHandleVector[pos2], newHandleVector[pos3+1], newHandleVector[pos3]);
				// Add the controllPoints to the face
				bezierTriangleMesh_.data(faceHandle).points(std::vector<Point>({ cp0, cp1, cp2, cp3, cp4, cp5 }));
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
	}
}

//-----------------------------------------------------------------------------

template <class MeshT>
int BezierTriangleMeshNode<MeshT>::pointsBefore(int level)
{
	// TODO das ist langsam?!
	int sum = 0;
	for (int i = 0; i < level; i++)
	{
		sum += grad() + 1 - i;
	}
	return sum;
}

//-----------------------------------------------------------------------------

template <class MeshT>
BezierTMesh::Point BezierTriangleMeshNode<MeshT>::getCP(
	int i, int j, int k, BezierTMesh::FaceHandle fh
)
{
	int cpIndex = pointsBefore(i) + j;

	auto faceControlP = bezierTriangleMesh_.data(fh);
	return faceControlP.controlPoint(cpIndex);
}

//-----------------------------------------------------------------------------

template <class MeshT>
BezierTMesh::Point BezierTriangleMeshNode<MeshT>::oneEntry(
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

	Point entry = FACTORIALS[grad()] / (FACTORIALS[i] * FACTORIALS[j] * FACTORIALS[k])
		* pow(baryCoords[0], i) * pow(baryCoords[1], j) * pow(baryCoords[2], k)
		* getCP(i, j, k, fh);
	return entry;
}

//-----------------------------------------------------------------------------

template <class MeshT>
BezierTMesh::Point BezierTriangleMeshNode<MeshT>::newPosition(
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
	for (int i = 0; i <= grad(); i++)
	{
		for (int j = 0; j + i <= grad(); j++)
		{
			for (int k = grad() - i - j; k + j + i == grad() && k >= 0; k++)
			{
				sum += oneEntry(i, j, k, baryCoords, fh);
			}
		}
	}
	return sum;
}

//-----------------------------------------------------------------------------

template <class MeshT>
BezierTMesh::Point BezierTriangleMeshNode<MeshT>::getFaceNormal(
	double u, double v,
	BezierTMesh::Point toEval, BezierTMesh::FaceHandle face,
	BezierTMesh::Point start
)
{
	Point pos1;
	Point pos2;

	// It is nessessary to distinguish between vertices that are
	// in the face and those which are on a border, because for
	// these the vector needs to show in the other direction and
	// sometimes the order for the cross-product needs to be changed
	// Otherwise the result is wrong i.e. flat triangle
	if (toEval[2] < STEPSIZE) {
		if (toEval[0] < STEPSIZE) {
			toEval = betri::getBaryCoords(u + STEPSIZE, v - STEPSIZE);
			pos2 = newPosition(toEval, face);
			toEval = betri::getBaryCoords(u, v - STEPSIZE);
			pos1 = newPosition(toEval, face);
		} else if (toEval[1] < STEPSIZE) {
			toEval = betri::getBaryCoords(u - STEPSIZE, v + STEPSIZE);
			pos1 = newPosition(toEval, face);
			toEval = betri::getBaryCoords(u - STEPSIZE, v);
			pos2 = newPosition(toEval, face);
		} else {
			toEval = betri::getBaryCoords(u - STEPSIZE, v);
			pos1 = newPosition(toEval, face);
			toEval = betri::getBaryCoords(u, v - STEPSIZE);
			pos2 = newPosition(toEval, face);
		}
	} else {
		toEval = betri::getBaryCoords(u + STEPSIZE, v);
		pos1 = newPosition(toEval, face);
		toEval = betri::getBaryCoords(u, v + STEPSIZE);
		pos2 = newPosition(toEval, face);
	}

	return cross(normalize(pos1 - start), normalize(pos2 - start));
}

//-----------------------------------------------------------------------------

template <class MeshT>
BezierTMesh::Point BezierTriangleMeshNode<MeshT>::getVertexNormal(
	double u, double v,
	BezierTMesh::FaceHandle face,
	BezierTMesh::Point start
)
{
	Point normal = Point(0.0);
	Point pos1;
	Point pos2;
	Point toEval1;
	Point toEval2;
	int count = 0;

	const MeshT::Color offsets[] = {
		MeshT::Color(-STEPSIZE, 0.0, 0.0, -STEPSIZE),
		MeshT::Color(0.0, -STEPSIZE, STEPSIZE, -STEPSIZE),
		MeshT::Color(STEPSIZE, -STEPSIZE, STEPSIZE, 0.0),
		MeshT::Color(STEPSIZE, 0.0, 0.0, STEPSIZE),
		MeshT::Color(0.0, STEPSIZE, -STEPSIZE, STEPSIZE),
		MeshT::Color(-STEPSIZE, STEPSIZE, -STEPSIZE, 0.0)
	};

	// loop over all neighbouring faces
	for (int i = 0; i < 6; i++) {
		toEval1 = betri::getBaryCoords(u + offsets[i][0], v + offsets[i][1]);
		toEval2 = betri::getBaryCoords(u + offsets[i][2], v + offsets[i][3]);

		// if the vector-points are in the 0 to 1 range (bary-coords)
		if (toEval1[0] >= 0.0 && toEval1[1] >= 0.0 && toEval1[2] >= 0.0 &&
			toEval2[0] >= 0.0 && toEval2[1] >= 0.0 && toEval2[2] >= 0.0 &&
			toEval1[0] + toEval1[1] + toEval1[2] <= 1.0 &&
			toEval2[0] + toEval2[1] + toEval2[2] <= 1.0
			) {
			pos1 = newPosition(toEval1, face);
			pos2 = newPosition(toEval2, face);

			normal += cross(normalize(pos1 - start), normalize(pos2 - start));
			count++;
		}
	}

	assert(count > 0);
	normal /= count;

	return normal;
}

//-----------------------------------------------------------------------------

/**
 * Create an VBO from the given Mesh object and apply CPU-tesselation
 * to the mesh without changing it representation as an BezierTriangleMesh.
 * (Only the VBO contains the additional vertex data)
 */
template <class MeshT>
void BezierTriangleMeshNode<MeshT>::VBOtesselatedFromMesh() {
	// create vertex buffer
	int vertexCount = bezierTriangleMesh_.n_faces() * VERTEXSUM;

	GLsizeiptr vboSize = vertexCount * surfaceDecl_.getVertexStride(); // bytes
	std::vector<float> vboData(vboSize); // float: 4 bytes

	// write counter
	int elementOffset = 0;

	float eps = 0.001;
	int i = 0;
	Point pos;
	Point n0, n1, n2, normal; //TODO BezierTMesh::Normal
	//Vec4uc vecCol(0, 0, 0, 1);
	Point vecCol(0, 0, 1);
	OpenMesh::VectorT<float, 2> texCoord; // TODO haben wir nicht brauchen wir noch
	for (auto face : bezierTriangleMesh_.faces()) {
		auto vertexHandle = bezierTriangleMesh_.fv_begin(face);
		auto vh0 = *(vertexHandle++);
		auto vh1 = *(vertexHandle++);
		auto vh2 = *(vertexHandle++);

		for (double u = 0.0; u <= 1.0; u += STEPSIZE) {
			for (double v = 0.0; u + v <= 1.0; v += STEPSIZE) {

				// Get the 3D-position Barycentric coords
				auto toEval = betri::getBaryCoords(u, v);

				//////////////
				// Position //
				//////////////
				auto resultPoint = newPosition(toEval, face);
				for (int m = 0; m < 3; ++m)
					vboData[elementOffset++] = float(resultPoint[m]);

				/////////////
				// Normale //
				/////////////
				// TODO face vs Vertex normals
				//normal = getFaceNormal(u, v, toEval, face, resultPoint);
				normal = getVertexNormal(u, v, face, resultPoint);
				//-------------------------------------------------------------

				for (int m = 0; m < 3; ++m)
					vboData[elementOffset++] = float(normal[m]);

				////////
				// UV //
				////////
				// store texcoord
				//texCoord = bezierTriangleMesh_.texcoord2D(v); // TODO
				//vboData[elementOffset++] = texCoord[0];
				//vboData[elementOffset++] = texCoord[1];
				vboData[elementOffset++] = 1.0;
				vboData[elementOffset++] = 0.0;

				///////////
				// Color //
				///////////

				// TODO use ints instead of floats
				MeshT::Color color;
				if (bezierTriangleMesh_.has_vertex_colors()) {// && props->colorSource() == DrawModes::COLOR_PER_VERTEX) {
					auto c0 = bezierTriangleMesh_.color(vh0);
					auto c1 = bezierTriangleMesh_.color(vh1);
					auto c2 = bezierTriangleMesh_.color(vh2);
					color = c0 * u + c1 * v + c2 * (1 - u - v);
				} else if (bezierTriangleMesh_.has_face_colors()) {// && props->colorSource() == DrawModes::COLOR_PER_FACE) {
					color = bezierTriangleMesh_.color(face);
				}
				for (int m = 0; m < 3; ++m) {
					vboData[elementOffset++] = float(color[m]);
				}
				vboData[elementOffset++] = float(1.0);

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

//-----------------------------------------------------------------------------

/**
 * Create a simple VBO from this Mesh.
 */
template <class MeshT>
void BezierTriangleMeshNode<MeshT>::VBOfromMesh() {
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
			//normal = bezierTriangleMesh_.normal(v);
			normal = bezierTriangleMesh_.normal(v);
			for (int m = 0; m < 3; ++m)
				vboData[elementOffset++] = float(normal[m]);

			// store texcoord
			//texCoord = bezierTriangleMesh_.texcoord2D(v); // TODO
			//vboData[elementOffset++] = texCoord[0];
			//vboData[elementOffset++] = texCoord[1];
			vboData[elementOffset++] = 1.0;
			vboData[elementOffset++] = 0.0;

			// TODO use ints instead of floats
			MeshT::Color color;
			if (bezierTriangleMesh_.has_vertex_colors()) {
				color = bezierTriangleMesh_.color(v);
			} else if (bezierTriangleMesh_.has_face_colors()) {
				color = bezierTriangleMesh_.color(face);
			}
			for (int m = 0; m < 3; ++m) {
				vboData[elementOffset++] = float(color[m]);
			}
			vboData[elementOffset++] = float(1.0);

			/* DrawMeshT<Mesh>::readVertex(
			byteCol[col] = (unsigned char)(vecCol[0]);
			byteCol[col] |= ((unsigned char)(vecCol[1])) << 8;
			byteCol[col] |= ((unsigned char)(vecCol[2])) << 16;
			byteCol[col] |= ((unsigned char)(vecCol[3])) << 24;
			*/
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

	for (int idxOffset = 0; idxOffset < numIndices; ++idxOffset)
	{
		iboData[idxOffset] = idxOffset;
	}

	// TODO i think the numIndices should be *4 and the numIndices-count itself is wrong, try to compare it with idxOffset
	if (numIndices)
		surfaceIBO_.upload(numIndices, &iboData[0], GL_STATIC_DRAW);

	surfaceIndexCount_ = numIndices;

	invalidateSurfaceMesh_ = false;
}

//-----------------------------------------------------------------------------

/**
 * Create a simple VBO from this Mesh.
 */
template <class MeshT>
void BezierTriangleMeshNode<MeshT>::VBOfromBoundingMesh()
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

	const int controlPointsPerFace = cpSum();
	for (FaceHandle face : bezierTriangleMesh_.faces()) {

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
					grad(),
					vboIndex,
					iboIndex,
					face_index,
					vboData,
					iboData,
					cpArray
				);
				break;
			}
			case betri::boundingVolumeType::ConvexHull:
			{
				// TODO is this the correct way to call this?
				betri::addConvexHullFromPoints(
					controlPointsPerFace,
					grad(),
					vboIndex,
					iboIndex,
					face_index,
					vboData,
					iboData,
					cpArray
				);
				break;
			}
			case betri::boundingVolumeType::BoundingMesh:
			{
				// TODO is this the correct way to call this?
				betri::addBoundingMeshFromPoints(
					controlPointsPerFace,
					grad(),
					vboIndex,
					iboIndex,
					face_index,
					vboData,
					iboData,
					cpArray
				);
				break;
			}
			case betri::boundingVolumeType::BoundingBillboard:
			{
				// TODO is this the correct way to call this?
				betri::addBoundingBillboardFromPoints(
					controlPointsPerFace,
					grad(),
					vboIndex,
					iboIndex,
					face_index,
					state_.eye(),// +state_.near_plane() * state_.viewing_direction(),
					state_.viewing_direction(),
					state_.near_plane(),
					state_.projection(),
					state_.modelview(),
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

///////////////////////////////////////////////////////////////////////////////
// Picking Functions
///////////////////////////////////////////////////////////////////////////////

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::pick(GLState& _state, PickTarget _target)
{
	setControlPointsColumnwise();
	updateGeometry();

	if (pick_texture_idx_ == 0)
		pick_init_texturing();

	ACG::GLState::disable(GL_COLOR_MATERIAL);

	switch (_target) {
		case PICK_VERTEX:
		{
			if (render_control_net_) {
				// TODO ist der Count hier richtig, was soll da überhaupt hin?
				_state.pick_set_maximum(bezierTriangleMesh_.n_vertices() * 2);
				pick_vertices(_state); // TODO tut das jetzt was ?
			}
			break;
		}

		case PICK_FACE:
		{
			_state.pick_set_maximum(1);
			pick_surface(_state, 0);
			break;
		}

		case PICK_SPLINE:
		{
			_state.pick_set_maximum(pick_texture_res_ * pick_texture_res_);
			pick_spline(_state);
			break;
		}

		case PICK_ANYTHING:
		{
			//_state.pick_set_maximum(bezierTriangleMesh_.n_control_points_m() * bezierTriangleMesh_.n_control_points_n() + 1);
			_state.pick_set_maximum(bezierTriangleMesh_.n_vertices() * 2 + 1);
			pick_vertices(_state);
			pick_surface(_state, bezierTriangleMesh_.n_vertices() * 2);
			break;
		}

		default:
			break;
	}
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::pick_vertices(GLState& _state)
{
	// radius in pixels
	int psize = 7;

	_state.pick_set_name(0);

	int face_id = 0;
	const int controlPointsPerFace = cpSum();
	for (auto &face : bezierTriangleMesh_.faces()) {
		auto faceControlP = bezierTriangleMesh_.data(face);
		Point cp;
		for (int i = 0; i < controlPointsPerFace; i++) {
			cp = faceControlP.controlPoint(i);

			_state.pick_set_name(face_id * controlPointsPerFace + i);

			// compute 3d radius of sphere
			Vec3d window_pos = _state.project((Vec3d)cp);
			int px = round(window_pos[0]);
			int py = round(window_pos[1]);

			double angle = acos(_state.viewing_direction(px, py).normalize() | _state.viewing_direction(px + psize, py).normalize());
			double l = (_state.eye() - (Vec3d)cp).norm();
			double r = l * tan(angle);

			// draw 3d sphere
			draw_sphere(cp, r, _state, sphere_);
		}
	}
	/*
	for (unsigned int i = 0; i < bezierTriangleMesh_.n_control_points_m(); ++i)
	{
		for (unsigned int j = 0; j < bezierTriangleMesh_.n_control_points_n(); ++j)
		{
			_state.pick_set_name(i * bezierTriangleMesh_.n_control_points_n() + j);

			// compute 3d radius of sphere
			Vec3d window_pos = _state.project((Vec3d)bezierTriangleMesh_(i, j));
			int px = round(window_pos[0]);
			int py = round(window_pos[1]);
			double angle = acos(_state.viewing_direction(px, py).normalize() | _state.viewing_direction(px + psize, py).normalize());
			double l = (_state.eye() - (Vec3d)bezierTriangleMesh_(i, j)).norm();
			double r = l * tan(angle);

			// draw 3d sphere
			draw_sphere(bezierTriangleMesh_(i, j), r, _state, sphere_);
		}
	}*/
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::pick_spline(
	GLState& _state
)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	ACG::GLState::enable(GL_TEXTURE_2D);

	ACG::GLState::disable(GL_COLOR_MATERIAL);
	ACG::GLState::disable(GL_LIGHTING);
	ACG::GLState::shadeModel(GL_FLAT);

	std::cout << "[BezierTriangle] pick_spline: \n"
		<< "pick_texture_baseidx_ = " << pick_texture_baseidx_
		<< ", _state.pick_current_index () = " << _state.pick_current_index()
		<< ", pick_texture_idx_ = " << pick_texture_idx_
		<< std::endl;

	if (_state.pick_current_index() != pick_texture_baseidx_) {
		pick_texture_baseidx_ = _state.pick_current_index();
		pick_create_texture(_state);
	} else {
		// do not blend colors (else color picking breaks!)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		// avoid aliasing at patch boundaries
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		// GL_REPLACE to avoid smearing colors (else color picking breaks!)
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

		ACG::GLState::bindTexture(GL_TEXTURE_2D, pick_texture_idx_);
	}

	drawSurface(_state);

	ACG::GLState::bindTexture(GL_TEXTURE_2D, 0);
	ACG::GLState::disable(GL_TEXTURE_2D);
	glPopAttrib();
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::pick_surface(
	GLState& _state, unsigned int _offset
)
{
	bool sampling_mode_backup = adaptive_sampling_;
	adaptive_sampling_ = false;

	// pick the whole surface
	_state.pick_set_name(_offset);
	drawSurface(_state);

	adaptive_sampling_ = sampling_mode_backup;
}

///////////////////////////////////////////////////////////////////////////////
// Unused Functions
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Update Functions
///////////////////////////////////////////////////////////////////////////////
template <class MeshT>
void BezierTriangleMeshNode<MeshT>::updateControlNetMeshSel()
{

	if (!invalidateControlNetMeshSel_)
		return;

	controlNetSelIBO_.del();
	/*
	if (bezierTriangleMesh_.controlpoint_selections_available())
	{

		int numU = bezierTriangleMesh_.n_control_points_m(),
			numV = bezierTriangleMesh_.n_control_points_n();

		// count # selected points
		int numSel = 0;
		for (int k = 0; k < numV; ++k)
		{
			for (int i = 0; i < numU; ++i)
			{
				if (bezierTriangleMesh_.controlpoint_selection(i, k))
					++numSel;
			}
		}

		// save count for draw call
		controlNetSelIndices_ = numSel;


		if (numSel)
		{
			// create array
			std::vector<int> iboData(numSel);
			numSel = 0;
			for (int k = 0; k < numV; ++k)
			{
				for (int i = 0; i < numU; ++i)
				{
					if (bezierTriangleMesh_.controlpoint_selection(i, k))
					{
						// see vertex indexing of vbo in updateControlNetMesh()
						// they are in "row-mayor" order
						iboData[numSel++] = k * numU + i;
					}
				}
			}

			controlNetSelIBO_.upload(numSel * 4, &iboData[0], GL_STATIC_DRAW);
		}

	}
	*/
	invalidateControlNetMeshSel_ = false;
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::updateControlPointSelectionTexture(
	GLState& _state
)
{
	create_cp_selection_texture(_state);
	controlPointSelectionTexture_valid_ = true;

	// also update index buffer for rendering selections
	invalidateControlNetMeshSel_ = true;
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::updateKnotVectorSelectionTexture(
	GLState& _state
)
{
	create_knot_selection_texture(_state);
	knotVectorSelectionTexture_valid_ = true;
}

///////////////////////////////////////////////////////////////////////////////
// Init Functions
///////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::selection_init_texturing(
	GLuint & _texture_idx
)
{
	// generate texture index
	glGenTextures(1, &_texture_idx);
	// bind texture as current
	ACG::GLState::bindTexture(GL_TEXTURE_2D, _texture_idx);
	// blend colors (otherwise lighting does not affect the texture)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	// avoid aliasing at patch boundaries
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	// GL_MODULATE to include lighting effects
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	// unbind current texture
	ACG::GLState::bindTexture(GL_TEXTURE_2D, 0);
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::create_cp_selection_texture(
	GLState& /*_state*/
)
{
	/*
	if (bezierTriangleMesh_.n_knots_m() == 0 || bezierTriangleMesh_.n_knots_n() == 0)
		return;

	if (cp_selection_texture_idx_ == 0)
		selection_init_texturing(cp_selection_texture_idx_);

	QImage b(cp_selection_texture_res_, cp_selection_texture_res_, QImage::Format_ARGB32);

	int degree_m = bezierTriangleMesh_.degree_m();
	int degree_n = bezierTriangleMesh_.degree_n();

	int numKnots_m = bezierTriangleMesh_.n_knots_m();
	int numKnots_n = bezierTriangleMesh_.n_knots_n();

	Knotvector knotvec_m = bezierTriangleMesh_.get_knotvector_m();
	Knotvector knotvec_n = bezierTriangleMesh_.get_knotvector_n();

	double minu = bezierTriangleMesh_.get_knot_m(degree_m);
	double maxu = bezierTriangleMesh_.get_knot_m(numKnots_m - degree_m - 1);
	double diffu = maxu - minu;

	double minv = bezierTriangleMesh_.get_knot_n(degree_n);
	double maxv = bezierTriangleMesh_.get_knot_n(numKnots_n - degree_n - 1);
	double diffv = maxv - minv;

	if (diffu == 0.0 || diffv == 0.0)
		return;

	int texelIdx_u = 0;

	for (int m = 0; m < cp_selection_texture_res_; ++m)
	{
		double step_m = (double)m / (double)cp_selection_texture_res_;
		double u = step_m * diffu;

		// get the span and check which knots are selected
		ACG::Vec2i span_u = bezierTriangleMesh_.spanm(u);
		// check for incomplete spline
		if (span_u[0] < 0 || span_u[1] < 0)
			return;

		// reset texture v coord for every new u coord
		int texelIdx_v = 0;

		// iterate over n direction
		for (int n = 0; n < cp_selection_texture_res_; ++n)
		{
			double step_n = double(n) / (double)cp_selection_texture_res_;
			double v = step_n * diffv;

			// get the span and check which knots are selected
			ACG::Vec2i span_v = bezierTriangleMesh_.spann(v);
			// check for incomplete spline
			if (span_v[0] < 0 || span_v[1] < 0)
				return;

			float alpha = 0.0; // blends between curve and highlight colors
			for (int i = 0; i < degree_m + 1; ++i) // degree+1 basis functions (those in the span) contribute
			{
				int idx_m = span_u[0] + i;

				for (int j = 0; j < degree_n + 1; ++j) // degree+1 basis functions (those in the span) contribute
				{
					int idx_n = span_v[0] + j;

					// basis functions sum up to 1. hence, we only have to sum up those with selected control point to get the blending weight
					if (bezierTriangleMesh_.controlpoint_selection(idx_m, idx_n))
						alpha += bezierTriangleMesh_.basisFunction(knotvec_m, idx_m, degree_m, u)
						* bezierTriangleMesh_.basisFunction(knotvec_n, idx_n, degree_n, v);
				}
			}

			// compute color
			Vec4f color = surface_color_ * (1.0 - alpha) + surface_highlight_color_ * alpha;

			// fill texture (switch v coord due to axis of texture image)
			b.setPixel(texelIdx_u, 255 - texelIdx_v, qRgba((int)(color[0] * 255.0), (int)(color[1] * 255.0), (int)(color[2] * 255.0), 255));

			++texelIdx_v;
		} // end of n direction iter

		++texelIdx_u;
	} // end of u direction iter


	// debug, output image
	// b.save("surfaceCPSelectionTexture.png", "PNG");

	cp_selection_texture_image_ = QGLWidget::convertToGLFormat(b);

	// bind texture
	ACG::GLState::bindTexture(GL_TEXTURE_2D, cp_selection_texture_idx_);
	glTexImage2D(GL_TEXTURE_2D,
		0, GL_RGBA, cp_selection_texture_image_.width(), cp_selection_texture_image_.height(),
		0, GL_RGBA, GL_UNSIGNED_BYTE, cp_selection_texture_image_.bits());
	*/
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::create_knot_selection_texture(
	GLState& _state
)
{
	/*
	if (bezierTriangleMesh_.n_knots_m() == 0 || bezierTriangleMesh_.n_knots_n() == 0)
		return;

	if (knot_selection_texture_idx_ == 0)
		selection_init_texturing(knot_selection_texture_idx_);

	QImage b(knot_selection_texture_res_, knot_selection_texture_res_, QImage::Format_ARGB32);

	int degree_m = bezierTriangleMesh_.degree_m();
	int degree_n = bezierTriangleMesh_.degree_n();

	int numKnots_m = bezierTriangleMesh_.n_knots_m();
	int numKnots_n = bezierTriangleMesh_.n_knots_n();

	Knotvector knotvec_m = bezierTriangleMesh_.get_knotvector_m();
	Knotvector knotvec_n = bezierTriangleMesh_.get_knotvector_n();

	double minu = bezierTriangleMesh_.get_knot_m(degree_m);
	double maxu = bezierTriangleMesh_.get_knot_m(numKnots_m - degree_m - 1);
	double diffu = maxu - minu;

	double minv = bezierTriangleMesh_.get_knot_n(degree_n);
	double maxv = bezierTriangleMesh_.get_knot_n(numKnots_n - degree_n - 1);
	double diffv = maxv - minv;

	if (diffu == 0.0 || diffv == 0.0)
		return;

	int texelIdx_u = 0;

	// if a knot is selected, select all knots in the span of this knot, too
	std::vector<bool> selectedKnotSpans_m(numKnots_m, false);
	for (int i = 0; i < numKnots_m; ++i)
	{
		if (bezierTriangleMesh_.get_knotvector_m_ref()->selection(i))
		{
			// get the span and check which knots are selected
			ACG::Vec2i span = bezierTriangleMesh_.spanm(bezierTriangleMesh_.get_knot_m(i));
			// check for incomple spline
			if (span[0] < 0 || span[1] < 0)
				return;

			for (int j = span[0]; j <= span[1] + degree_m; ++j)
				selectedKnotSpans_m[j] = true;
		}
	}
	//   std::cout << "selectedKnotSpans_m: " << std::flush;
	//   for (unsigned int i = 0; i < selectedKnotSpans_m.size(); ++i)
	//     std::cout << selectedKnotSpans_m[i] << ", " << std::flush;
	//   std::cout << std::endl;


	std::vector<bool> selectedKnotSpans_n(numKnots_n, false);
	for (int i = 0; i < numKnots_n; ++i)
	{
		if (bezierTriangleMesh_.get_knotvector_n_ref()->selection(i))
		{
			// get the span and check which knots are selected
			ACG::Vec2i span = bezierTriangleMesh_.spann(bezierTriangleMesh_.get_knot_n(i));
			// check for incomple spline
			if (span[0] < 0 || span[1] < 0)
				return;

			for (int j = span[0]; j <= span[1] + degree_n; ++j)
				selectedKnotSpans_n[j] = true;
		}
	}
	//   std::cout << "selectedKnotSpans_n: " << std::flush;
	//   for (unsigned int i = 0; i < selectedKnotSpans_n.size(); ++i)
	//     std::cout << selectedKnotSpans_n[i] << ", " << std::flush;
	//   std::cout << std::endl;


	for (int m = 0; m < knot_selection_texture_res_; ++m)
	{
		double step_m = (double)m / (double)knot_selection_texture_res_;
		double u = step_m * diffu;

		Vec2i interval_m = bezierTriangleMesh_.interval_m(u);

		// reset texture v coord for every new u coord
		int texelIdx_v = 0;

		for (int n = 0; n < knot_selection_texture_res_; ++n)
		{
			double step_n = (double)n / (double)knot_selection_texture_res_;
			double v = step_n * diffv;

			Vec2i interval_n = bezierTriangleMesh_.interval_n(v);

			// check if highlighted
			bool selected_m = (selectedKnotSpans_m[interval_m[0]] && selectedKnotSpans_m[interval_m[1]]);
			bool selected_n = (selectedKnotSpans_n[interval_n[0]] && selectedKnotSpans_n[interval_n[1]]);

			Vec4f color;
			if (selected_m && selected_n)
				//         color = _state.specular_color();
				//         color = _state.base_color();
				color = surface_highlight_color_;
			else if ((selected_m && !selected_n) || (!selected_m && selected_n))
				//         color = _state.ambient_color() *0.5 + _state.specular_color() * 0.5;
				//         color = _state.base_color() *0.5 + _state.specular_color() * 0.5;
				color = surface_highlight_color_ * 0.5 + surface_color_ * 0.5;
			else
				//         color = _state.ambient_color() *0.5 + _state.diffuse_color() * 0.5;
				//         color = _state.base_color() *0.5 + _state.diffuse_color() * 0.5;
				color = surface_color_;


			// fill texture
			b.setPixel(texelIdx_u, 255 - texelIdx_v, qRgba((int)(color[0] * 255.0), (int)(color[1] * 255.0), (int)(color[2] * 255.0), 255));

			++texelIdx_v;
		} // end of v direction

		++texelIdx_u;
	} // end of u direction

	// debug, output image
	// b.save("surfaceKnotSelectionTexture.png", "PNG");

	knot_selection_texture_image_ = QGLWidget::convertToGLFormat(b);

	// bind texture
	ACG::GLState::bindTexture(GL_TEXTURE_2D, knot_selection_texture_idx_);
	glTexImage2D(GL_TEXTURE_2D,
		0, GL_RGBA, knot_selection_texture_image_.width(), knot_selection_texture_image_.height(),
		0, GL_RGBA, GL_UNSIGNED_BYTE, knot_selection_texture_image_.bits());
	*/
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::pick_init_texturing()
{
	std::cout << "[BSplineSurface] pick_init_texturing()" << std::endl;

	pick_texture_res_ = 256;
	pick_texture_baseidx_ = 0;

	// generate texture index
	glGenTextures(1, &pick_texture_idx_);
	// bind texture as current
	ACG::GLState::bindTexture(GL_TEXTURE_2D, pick_texture_idx_);
	// do not blend colors (else color picking breaks!)
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	// avoid aliasing at patch boundaries
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	// GL_REPLACE to avoid smearing colors (else color picking breaks!)
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	// unbind current texture
	ACG::GLState::bindTexture(GL_TEXTURE_2D, 0);
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::pick_create_texture(GLState& _state)
{
	std::cout << "[BSplineSurface] pick_create_texture()" << std::endl;

	QImage b(pick_texture_res_, pick_texture_res_, QImage::Format_ARGB32);
	QImage texture(pick_texture_res_, pick_texture_res_, QImage::Format_ARGB32);

	// fill with colors
	int cur_idx = 0;
	for (int i = 0; i < pick_texture_res_; ++i) {
		for (int j = pick_texture_res_ - 1; j >= 0; j--) {
			Vec4uc cur_col(_state.pick_get_name_color(cur_idx));
			b.setPixel(i, j, qRgba((int)cur_col[0], (int)cur_col[1], (int)cur_col[2], (int)cur_col[3]));

			Vec4f testcol = Vec4f((float)cur_col[0], (float)cur_col[1], (float)cur_col[2], (float)cur_col[3]);
			texture.setPixel(i, j, qRgba((int)(testcol[0] * 255.0), (int)(testcol[1] * 255.0), (int)(testcol[2] * 255.0), 255));
			cur_idx++;
		}
	}

	/*
	  // creates checkerboard texture for debugging purposes

	  bool odd_row = true;
	  bool odd_col = true;
	  bool green = true;
	  for( int i = 0; i < pick_texture_res_; ++i)
	  {
		if (i % 20 == 0)
		  odd_row = !odd_row;

		odd_col = true;
		for( int j = 0; j < pick_texture_res_; ++j)
		{
		  if (j % 20 == 0)
			odd_col = !odd_col;

		  green = (odd_row && odd_col) || (!odd_row && !odd_col);

		  if (green)
			b.setPixel (i, j, qRgba(0, 255, 0, 255));
		  else
			b.setPixel (i, j, qRgba(255, 0, 255, 255));
		}
	  }
	*/

	// debug, output image
  //   b.save("surfacePickingTexture.png", "PNG");
	texture.save("surfacePickingTexture.png", "PNG");

	pick_texture_image_ = QGLWidget::convertToGLFormat(b);

	// bind texture
	ACG::GLState::bindTexture(GL_TEXTURE_2D, pick_texture_idx_);
	glTexImage2D(GL_TEXTURE_2D,
		0, GL_RGBA, pick_texture_image_.width(), pick_texture_image_.height(),
		0, GL_RGBA, GL_UNSIGNED_BYTE, pick_texture_image_.bits());
}

//-----------------------------------------------------------------------------

template <class MeshT>
ACG::Vec4f BezierTriangleMeshNode<MeshT>::generateHighlightColor(
	ACG::Vec4f _color
)
{
	float c1 = _color[0] * 1.5;
	c1 = c1 > 255.0 ? 255 : c1;

	float c2 = _color[1] * 1.5;
	c2 = c2 > 255.0 ? 255 : c2;

	float c3 = _color[2] * 1.5;
	c3 = c3 > 255.0 ? 255 : c3;

	return Vec4f(c1, c2, c3, _color[3]);
}

//-----------------------------------------------------------------------------

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::set_arb_texture(
	const QImage& _texture, bool _repeat, float _u_repeat, float _v_repeat
)
{
	if (arb_texture_idx_ == 0)
		selection_init_texturing(arb_texture_idx_);

	ACG::GLState::bindTexture(GL_TEXTURE_2D, 0);

	arb_texture_repeat_ = _repeat;
	arb_texture_repeat_u_ = _u_repeat;
	arb_texture_repeat_v_ = _v_repeat;

	arb_texture_image_ = QGLWidget::convertToGLFormat(_texture);
	int u_res = arb_texture_image_.width();
	int v_res = arb_texture_image_.height();

	// bind texture as current
	ACG::GLState::bindTexture(GL_TEXTURE_2D, arb_texture_idx_);

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	glTexImage2D(GL_TEXTURE_2D,
		0, GL_RGBA, u_res, v_res,
		0, GL_RGBA, GL_UNSIGNED_BYTE, arb_texture_image_.bits());

	// unbind current texture
	ACG::GLState::bindTexture(GL_TEXTURE_2D, 0);
}

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
