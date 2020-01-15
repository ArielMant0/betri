#define ACG_BEZIERTRIANGLEMESHNODE_C

///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////
#include "BezierTriangleMeshNode.hh"
#include <ACG/GL/gl.hh>
#include <ACG/GL/GLError.hh>
#include <ACG/GL/IRenderer.hh>
#include <ACG/GL/Benchmarker.hh>
#include <ACG/Utils/VSToolsT.hh>
#include <vector>

#include <iostream>
#include <fstream>

#include "globals/BezierOptions.hh"
#include "BezierMathUtil.hh"
#include "boundVol/BVolGenerator.hh"
#include "RayCastingShaderGenerator.hh"

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
	// https://github.com/progschj/OpenGL-Examples/blob/master/10queries_conditional_render.cpp
	// https://community.khronos.org/t/how-to-measure-timing-in-opengl/61544/22
	// https://stackoverflow.com/questions/28530798/how-to-make-a-basic-fps-counter
	std::clock_t start = std::clock();

	// TODO this should propably be done differently
	state_ = &_state;

	// only render mesh if that is possible (e.g. has control points)
	if (!bezierTriangleMesh_.isRenderable())
		return;


	// check if textures are still valid
	if (bspline_selection_draw_mode_ == CONTROLPOINT
		&& controlPointSelectionTexture_valid_ == false)
		updateControlPointSelectionTexture(_state);
	if (bspline_selection_draw_mode_ == KNOTVECTOR
		&& knotVectorSelectionTexture_valid_ == false)
		updateKnotVectorSelectionTexture(_state);

	int renderOption = betri::option(betri::BezierOption::TESSELLATION_TYPE);
	int bVolume = betri::option(betri::BezierOption::BOUNDING_VOLUME);

	if (Benchmarker::instance()->active()) {
		renderOption = Benchmarker::instance()->renderMode();
		bVolume = Benchmarker::instance()->bVolume();
	}

	if (Benchmarker::instance()->update() || (
			bVolume == betri::boundingVolumeType::BoundingBillboard && 
			renderOption == betri::TESSELLATION_TYPE::RAYTRACING
		)
	)
		invalidateSurfaceMesh_ = true;

	size_t layer = 0;
	for (size_t i = 0; i < _drawMode.getNumLayers(); ++i) {
		if (_drawMode.getLayer(i)->primitive() == DrawModes::PRIMITIVE_POLYGON)
			layer = i;
	}
	// TODO is this a good way to do it?
	if (drawModeProps_ != *(_drawMode.getLayer(layer))) {
		// TODO dont update always
		invalidateSurfaceMesh_ = true;

		drawModeProps_ = DrawModes::DrawModeProperties(
			_drawMode.getLayer(layer)->primitive(),
			_drawMode.getLayer(layer)->lightStage(),
			_drawMode.getLayer(layer)->normalSource(),
			_drawMode.getLayer(layer)->colorSource(),
			_drawMode.getLayer(layer)->texcoordSource()
		);
	}

	updateSurfaceMesh(renderOption);

	RenderObject ro;
	ro.debugName = "BTMeshNode"; // TODO geht das so?

	for (size_t i = 0; i < _drawMode.getNumLayers(); ++i) {
		const DrawModes::DrawModeProperties* props = _drawMode.getLayer(i);

		// TODO this can propably be done differently
		//ACG::GLState::enable(GL_CULL_FACE);
		//ACG::GLState::cullFace(GL_BACK);
		//std::cerr << bool(glIsEnabled(GL_CULL_FACE)) << " " << _state.isStateEnabled(GL_CULL_FACE) << std::endl;
		if (betri::option(betri::BezierOption::CULL_FACES) > 0 &&
			(
				renderOption != betri::TESSELLATION_TYPE::RAYTRACING ||
				bVolume != betri::boundingVolumeType::BoundingBillboard
			)
		) {
			//glFrontFace(GL_CW); // TODO
			_state.enable(GL_CULL_FACE);
			if (renderOption == betri::TESSELLATION_TYPE::CPU)
				_state.cullFace(GL_FRONT);
			else
				_state.cullFace(GL_BACK);
		} else 
			_state.disable(GL_CULL_FACE);

		ro.initFromState(&_state);
		ro.setupShaderGenFromDrawmode(props);
		ro.depthTest = true;

		///////////////////////////////////////////////////////////////////////
		//
		///////////////////////////////////////////////////////////////////////
		// generated texcoords for environment mapping should be computed in fragment shader,
		// because normals aren't available in the vertex shader
		ro.shaderDesc.texGenPerFragment = true;

		//	ro.addTexture(ACG::RenderObject::Texture(arb_texture_idx_), 0);
		if (props->textured())// && arb_texture_idx_)
			ro.addTexture(RenderObject::Texture(checkerBoardTex_.id(), GL_TEXTURE_2D), 1, true);

		if (props->primitive() == DrawModes::PRIMITIVE_POINT ||
			props->primitive() == DrawModes::PRIMITIVE_POLYGON ||
			props->primitive() == DrawModes::PRIMITIVE_WIREFRAME ||
			props->primitive() == DrawModes::PRIMITIVE_HIDDENLINE)
		{
			ro.vertexBuffer = surfaceVBO_.id();
			ro.indexBuffer = surfaceIBO_.id();
			ro.vertexDecl = &surfaceDecl_;

			if (props->primitive() == DrawModes::PRIMITIVE_WIREFRAME ||
				props->primitive() == DrawModes::PRIMITIVE_HIDDENLINE)
				ro.fillMode = GL_LINE;
			else
				ro.fillMode = GL_FILL;

			GLenum roPrimitives = GL_TRIANGLES;

			if (props->primitive() == DrawModes::PRIMITIVE_POINT) {
				//ro.setupPointRendering(_mat->pointSize(), Vec2f((float)_state.viewport_width(), (float)_state.viewport_height()));
				//applyRenderObjectSettings(props->primitive(), &ro);

				//roPrimitives = GL_POINTS;
				//ro.fillMode = GL_POINT;
			}

			if (renderOption == betri::TESSELLATION_TYPE::RAYTRACING) {

				// TODO this is a doublication
				if (!controlPointTex_.is_valid()) {
					updateTexBuffers();
				}

				///////////////////////
				// Additional States //
				///////////////////////
				ro.shaderDesc.shadeMode = SG_SHADE_UNLIT;

				////////////////
				// Set Shader //
				////////////////
				ro.shaderDesc.vertexTemplateFile = "BezierTriangle/vertex.glsl";
				//ro.shaderDesc.fragmentTemplateFile = "BezierTriangle/fragment.glsl";
				ro.shaderDesc.fragmentTemplateFile = "BezierTriangle/raytracing_gen_fs.glsl";

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
				else if (betri::option(betri::BezierOption::VISUALISATION_MODE) == betri::VIS_MODE::UV)
					shaderMacro.sprintf("#define SG_OUTPUT_UV");
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

				if (drawModeProps_.texcoordSource() != DrawModes::DrawModeTexCoordSource::TEXCOORD_NONE) {
					shaderMacro.sprintf("#define TEXTURED");
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

				ro.setUniform("controlPointTex", int(0));
				ro.addTexture(RenderObject::Texture(controlPointTex_.id(), GL_TEXTURE_2D), 0, false);

				if (drawModeProps_.texcoordSource() != DrawModes::DrawModeTexCoordSource::TEXCOORD_NONE) {
					ro.setUniform("uvCoordTex", int(1));
					ro.addTexture(RenderObject::Texture(texCoordTex_.id(), GL_TEXTURE_2D), 1, false);

					ro.setUniform("exampleTex", int(2));
					ro.addTexture(RenderObject::Texture(checkerBoardTex_.id(), GL_TEXTURE_2D), 2, false);
				}
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

				// Textures ---------------------------------------------------
				ro.setUniform("controlPointTex", int(0));
				ro.addTexture(RenderObject::Texture(controlPointTex_.id(), GL_TEXTURE_2D), 0, false);
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
	}


	///////////////////////////////////////////////////////////////////////////
	// draw the control net (includes selection on the net)
	///////////////////////////////////////////////////////////////////////////

	// TODO
	if (render_control_net())
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

	// TODO
	NEWVERTICES = betri::mersennePrime(ITERATIONS);
	VERTEXSUM = betri::gaussSum(NEWVERTICES + 2);
	STEPSIZE = 1.0 / (double(NEWVERTICES) + 1.0);

	// reset number of control points
	cpSum_ = -1;

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
		if (Benchmarker::instance()->active()) // TODO
			renderOption = Benchmarker::instance()->renderMode();

		surfaceDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_POSITION);
		// TODO the normal is not needed for raytracing but i was not able to remove it from the shader
		if (renderOption != betri::TESSELLATION_TYPE::RAYTRACING) {
			surfaceDecl_.addElement(GL_FLOAT, 3, VERTEX_USAGE_NORMAL);
		}
		if (drawModeProps_.textured() || renderOption == betri::TESSELLATION_TYPE::RAYTRACING) {
			surfaceDecl_.addElement(GL_FLOAT, 2, VERTEX_USAGE_TEXCOORD);
		}
		if (drawModeProps_.colored() && renderOption != betri::TESSELLATION_TYPE::RAYTRACING) {
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
void BezierTriangleMeshNode<MeshT>::updateSurfaceMesh(const int meshOption)
{
	// TODO this is dump
	if (!invalidateSurfaceMesh_)
		return;

	updateRaycastingFormula(
		grad(),
		OpenFlipper::Options::shaderDir().absolutePath().toLocal8Bit().constData()
	);

	//if (betri::option(betri::BezierOption::BOUNDING_VOLUME) != betri::boundingVolumeType::BoundingBillboard || i == 0) {
		surfaceVBO_.del();
		surfaceIBO_.del();
	
		updateSurfaceDecl();
	//}

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

	// TODO Hä?
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
		controlNetVBO_.upload(vboSize, vboData.data(), GL_STATIC_DRAW);

	vboData.clear();

	// TODO more tests that this is correct for all cases and that the index counts are corrects (idxOffset vs numIndices)

	int bottomTriangles = betri::gaussSum(grad());
	// TODO unterschiedliche Faces können unterschiedliche kontrollpunkte haben auch wenn sie aneinanderliegen?! deswegen mehrere Linien an der grenze ?
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
		controlNetLineIBO_.upload(numIndices * 4, iboData.data(), GL_STATIC_DRAW);

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

		// write counter
		int elementOffset = 0;
		for (FaceHandle face : bezierTriangleMesh_.faces()) {
			auto &faceControlP = bezierTriangleMesh_.data(face);
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

	MeshT::TexCoord2D texCoord;
	const int texCoordsPerFace = 3;
	const size_t texCoordBufSize = texCoordsPerFace * bezierTriangleMesh_.n_faces();

	if (texCoordBufSize && drawModeProps_.texcoordSource() != DrawModes::DrawModeTexCoordSource::TEXCOORD_NONE) {
		std::vector<float> texCoordBuf(texCoordBufSize * 2);

		// write counter
		int elementOffset = 0;
		for (auto &face : bezierTriangleMesh_.faces()) {
			for (auto v = bezierTriangleMesh_.fv_begin(face); v != bezierTriangleMesh_.fv_end(face); ++v) {

				texCoord = bezierTriangleMesh_.texcoord2D(v); // TODO
				for (int m = 0; m < 2; ++m)
					texCoordBuf[elementOffset++] = texCoord[m];
			}
		}

		texCoordTex_.bind();
		texCoordTex_.parameter(GL_TEXTURE_MIN_FILTER, GL_NEAREST); // disable filtering
		texCoordTex_.parameter(GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		texCoordTex_.setData(0, GL_RG32F, texCoordsPerFace, bezierTriangleMesh_.n_faces(), GL_RG, GL_FLOAT, &texCoordBuf[0]);
	}

#endif
}

///////////////////////////////////////////////////////////////////////////////
// Functions for VBO creation
///////////////////////////////////////////////////////////////////////////////
template <class MeshT>
BezierTMesh::Point BezierTriangleMeshNode<MeshT>::getFaceNormal(
	double u, double v,
	BezierTMesh::Point toEval, BezierTMesh::FaceHandle face,
	BezierTMesh::Point start
)
{
	Point pos1;
	Point pos2;

	auto &cps = mesh().data(face).points();

	// It is nessessary to distinguish between vertices that are
	// in the face and those which are on a border, because for
	// these the vector needs to show in the other direction and
	// sometimes the order for the cross-product needs to be changed
	// Otherwise the result is wrong i.e. flat triangle
	if (toEval[2] < STEPSIZE) {
		if (toEval[0] < STEPSIZE) {
			toEval = betri::getBaryCoords(u + STEPSIZE, v - STEPSIZE);
			pos2 = betri::evalSurface(cps, toEval, grad());
			toEval = betri::getBaryCoords(u, v - STEPSIZE);
			pos1 = betri::evalSurface(cps, toEval, grad());
		} else if (toEval[1] < STEPSIZE) {
			toEval = betri::getBaryCoords(u - STEPSIZE, v + STEPSIZE);
			pos1 = betri::evalSurface(cps, toEval, grad());
			toEval = betri::getBaryCoords(u - STEPSIZE, v);
			pos2 = betri::evalSurface(cps, toEval, grad());
		} else {
			toEval = betri::getBaryCoords(u - STEPSIZE, v);
			pos1 = betri::evalSurface(cps, toEval, grad());
			toEval = betri::getBaryCoords(u, v - STEPSIZE);
			pos2 = betri::evalSurface(cps, toEval, grad());
		}
	} else {
		toEval = betri::getBaryCoords(u + STEPSIZE, v);
		pos1 = betri::evalSurface(cps, toEval, grad());
		toEval = betri::getBaryCoords(u, v + STEPSIZE);
		pos2 = betri::evalSurface(cps, toEval, grad());
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

	auto &cps = mesh().data(face).points();

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
			pos1 = betri::evalSurface(cps, toEval1, grad());
			pos2 = betri::evalSurface(cps, toEval2, grad());

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
	std::vector<float> vboData(vboSize / 4); // float: 4 bytes

	// write counter
	int elementOffset = 0;

	Point pos;
	MeshT::Normal normal;
	MeshT::TexCoord2D texCoord; // TODO haben wir nicht brauchen wir noch
	for (auto face : bezierTriangleMesh_.faces()) {
		auto vertexHandle = bezierTriangleMesh_.fv_begin(face);
		auto vh0 = *(vertexHandle++);
		auto vh1 = *(vertexHandle++);
		auto vh2 = *(vertexHandle++);

		auto &cps = mesh().data(face).points();

		for (double u = 0.0; u <= 1.0; u += STEPSIZE) {
			for (double v = 0.0; u + v <= 1.0; v += STEPSIZE) {
				//////////////
				// Position //
				//////////////
				// Get the 3D-position Barycentric coords
				auto toEval = betri::getBaryCoords(u, v);
				pos = betri::evalSurface(cps, toEval, grad());
				for (int m = 0; m < 3; ++m)
					vboData[elementOffset++] = float(pos[m]);

				/////////////
				// Normals //
				/////////////
				// TODO face vs Vertex normals
				//normal = getFaceNormal(u, v, toEval, face, pos);
				normal = getVertexNormal(u, v, face, pos);

				for (int m = 0; m < 3; ++m)
					vboData[elementOffset++] = float(normal[m]);

				////////
				// UV //
				////////
				// store texcoord
				switch (drawModeProps_.texcoordSource()) {
					case DrawModes::DrawModeTexCoordSource::TEXCOORD_PER_VERTEX:
					{
						texCoord = u * bezierTriangleMesh_.texcoord2D(vh0)
							+ v * bezierTriangleMesh_.texcoord2D(vh1)
							+ (1 - u - v) * bezierTriangleMesh_.texcoord2D(vh2);
						vboData[elementOffset++] = texCoord[0];
						vboData[elementOffset++] = texCoord[1];
						break;
					}
					case DrawModes::DrawModeTexCoordSource::TEXCOORD_PER_HALFEDGE:
					{
						vboData[elementOffset++] = 1.0;
						vboData[elementOffset++] = 0.0;
						break;
					}
					default: break;
				}

				///////////
				// Color //
				///////////
				if (drawModeProps_.colorSource() != DrawModes::DrawModeColorSource::COLOR_NONE) {
					MeshT::Color color(0.0, 0.0, 0.0, 1.0);
					switch (drawModeProps_.colorSource()) {
						case DrawModes::DrawModeColorSource::COLOR_PER_VERTEX:
						{
							auto c0 = bezierTriangleMesh_.color(vh0);
							auto c1 = bezierTriangleMesh_.color(vh1);
							auto c2 = bezierTriangleMesh_.color(vh2);
							color = c0 * u + c1 * v + c2 * (1 - u - v);
							break;
						}
						case DrawModes::DrawModeColorSource::COLOR_PER_FACE:
						{
							color = bezierTriangleMesh_.color(face);
							break;
						}
						// TODO
						case DrawModes::DrawModeColorSource::COLOR_PER_EDGE:
						case DrawModes::DrawModeColorSource::COLOR_PER_HALFEDGE:
						default: break;
					}
						for (int m = 0; m < 4; ++m) {
							vboData[elementOffset++] = float(color[m]);
						}
				}

				/*
				// TODO use ints instead of floats
				Vec4uc vecCol(0, 0, 0, 1);
				DrawMeshT<Mesh>::readVertex(
				byteCol[col] = (unsigned char)(vecCol[0]);
				byteCol[col] |= ((unsigned char)(vecCol[1])) << 8;
				byteCol[col] |= ((unsigned char)(vecCol[2])) << 16;
				byteCol[col] |= ((unsigned char)(vecCol[3])) << 24;
				*/
			}
		}
	}

	if (vboSize)
		surfaceVBO_.upload(vboSize, vboData.data(), GL_STATIC_DRAW);

	vboData.clear();

	// create index buffer
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

	assert(idxOffset == numIndices);

	if (numIndices)
		surfaceIBO_.upload(numIndices * 4, iboData.data(), GL_STATIC_DRAW);

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

	//assert(bezierTriangleMesh_.n_vertices() == 6);
	//assert(bezierTriangleMesh_.n_edges() == 10);
	//assert(bezierTriangleMesh_.n_faces() == 5);

	GLsizeiptr vboSize = vertexCount * surfaceDecl_.getVertexStride(); // bytes
	std::vector<float> vboData(vboSize / 4); // float: 4 bytes

	// write counter
	int elementOffset = 0;

	Point pos;
	MeshT::Normal normal;
	MeshT::TexCoord2D texCoord;
	for (FaceHandle face : bezierTriangleMesh_.faces()) {
		for (auto v = bezierTriangleMesh_.fv_begin(face); v != bezierTriangleMesh_.fv_end(face); ++v) {
			//////////////
			// Position //
			//////////////
			pos = bezierTriangleMesh_.point(v);
			//std::cerr << "vertex pos " << pos << '\n';
			for (int m = 0; m < 3; ++m)
				vboData[elementOffset++] = float(pos[m]);

			/////////////
			// Normals //
			/////////////
			normal = bezierTriangleMesh_.normal(v);
			//std::cerr << "vertex normal " << normal << '\n';
			for (int m = 0; m < 3; ++m)
				vboData[elementOffset++] = float(normal[m]);

			////////
			// UV //
			////////
			// store texcoord
			switch (drawModeProps_.texcoordSource()) {
				case DrawModes::DrawModeTexCoordSource::TEXCOORD_PER_VERTEX:
				{
					texCoord = bezierTriangleMesh_.texcoord2D(v); // TODO
					vboData[elementOffset++] = texCoord[0];
					vboData[elementOffset++] = texCoord[1];
					break;
				}
				case DrawModes::DrawModeTexCoordSource::TEXCOORD_PER_HALFEDGE:
				{
					vboData[elementOffset++] = 1.0;
					vboData[elementOffset++] = 0.0;
					break;
				}
				default: break;
			}

			///////////
			// Color //
			///////////
			if (drawModeProps_.colorSource() != DrawModes::DrawModeColorSource::COLOR_NONE) {
				MeshT::Color color(0.0, 0.0, 0.0, 1.0);
				switch (drawModeProps_.colorSource()) {
					case DrawModes::DrawModeColorSource::COLOR_PER_VERTEX:
					{
						color = bezierTriangleMesh_.color(v);
						break;
					}
					case DrawModes::DrawModeColorSource::COLOR_PER_FACE:
					{
						color = bezierTriangleMesh_.color(face);
						break;
					}
					// TODO
					case DrawModes::DrawModeColorSource::COLOR_PER_EDGE:
					case DrawModes::DrawModeColorSource::COLOR_PER_HALFEDGE:
					default: break;
				}
				for (int m = 0; m < 4; ++m) {
					vboData[elementOffset++] = float(color[m]);
				}
			}
		}
	}

	if (vboSize)
		surfaceVBO_.upload(vboSize, vboData.data(), GL_STATIC_DRAW);

	vboData.clear();

	// create index buffer
	int numIndices = vertexCount;

	std::vector<int> iboData(numIndices);

	for (int idxOffset = 0; idxOffset < numIndices; ++idxOffset)
	{
		iboData[idxOffset] = idxOffset;
	}

	// TODO i think the numIndices should be *4 and the numIndices-count itself is wrong, try to compare it with idxOffset
	if (numIndices)
		surfaceIBO_.upload(numIndices * 4, iboData.data(), GL_STATIC_DRAW);

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
	///////////////////////
	// Setup VBO and IBO //
	///////////////////////

	// TODO different bounding volumes
	int bVolume = betri::option(betri::BezierOption::BOUNDING_VOLUME);
	if (Benchmarker::instance()->active())
		bVolume = Benchmarker::instance()->bVolume();
	/*
	// TODO
	std::cerr << Benchmarker::instance()->active() << " "
		<< Benchmarker::instance()->renderMode()
		<< Benchmarker::instance()->bVolume() << std::endl;*/

	int numVerts;
	int numIndices;
	betri::estimateVertexIndexCounts(bVolume, numVerts, numIndices);

	int vertexCount = bezierTriangleMesh_.n_faces() * numVerts;
	GLsizeiptr vboSize = vertexCount * surfaceDecl_.getVertexStride(); // bytes

	int indexCount = bezierTriangleMesh_.n_faces() * numIndices;

	// create index buffer
	std::vector<int> iboData;
	iboData.reserve(indexCount);
	// create vertex buffer
	std::vector<float> vboData;
	vboData.reserve(vboSize / 4);

	////////////////////////////////
	// Fill with boundingbox data //
	////////////////////////////////

	int vboIndex = 0;
	int iboIndex = 0;
	int face_index = 0;

	const int controlPointsPerFace = cpSum();
	for (FaceHandle face : bezierTriangleMesh_.faces()) {

		auto &faceControlP = bezierTriangleMesh_.data(face);
		std::vector<Point> cpArray = faceControlP.points();

		switch (bVolume) {
			case betri::boundingVolumeType::BoundingTetraeder:
			{
				// TODO is this the correct way to call this?
				betri::addBoundingTetraederFromPoints(
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
					state_->eye(), // TODO it kind of sucks that we need this vars here
					state_->viewing_direction(), // since only for that we need the state_
					state_->near_plane(),
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

	////////////////////////////////////
	// Upload VBO and IBO and cleanup //
	////////////////////////////////////
	constexpr int bytes = 4;
	vboSize = vboData.size() * bytes;
	indexCount = iboData.size() * bytes;

	// in bytes
	switch (bVolume) {
		case betri::boundingVolumeType::AABB:
		case betri::boundingVolumeType::PrismVolume:
		case betri::boundingVolumeType::BoundingTetraeder:
			assert(vboSize == vertexCount * surfaceDecl_.getVertexStride());
			assert(indexCount == bezierTriangleMesh_.n_faces() * numIndices * bytes);
			break;
		case betri::boundingVolumeType::ConvexHull:
		case betri::boundingVolumeType::BoundingBillboard:
			//assert(vboSize >= vertexCount * surfaceDecl_.getVertexStride());
			//assert(indexCount >= bezierTriangleMesh_.n_faces() * numIndices * bytes);
			break;
		default: break;
	}

	if (vboSize) {
		//if (bVolume == betri::boundingVolumeType::BoundingBillboard) {
			//if (i == 0)
		//		surfaceVBO_.upload(vboSize, vboData.data(), GL_STREAM_DRAW);
			//else
			//	surfaceVBO_.uploadSubData(0, vboSize, vboData.data());
			//i++;
		//}
		//else
			surfaceVBO_.upload(vboSize, vboData.data(), GL_STATIC_DRAW);
	}

	vboData.clear();

	if (indexCount)
		surfaceIBO_.upload(indexCount, iboData.data(), GL_STATIC_DRAW);

	iboData.clear();

	surfaceIndexCount_ = indexCount;

	invalidateSurfaceMesh_ = false;
}

///////////////////////////////////////////////////////////////////////////////
// Picking Functions
///////////////////////////////////////////////////////////////////////////////

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::pick(GLState& _state, PickTarget _target)
{
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


template <class MeshT>
void BezierTriangleMeshNode<MeshT>::createCheckerBoardTex()
{
	const int sizeX = 256;
	const int sizeY = 256;
	const size_t texSize = sizeX * sizeY;
	std::vector<float> texBuf(texSize * 3);

	int elementOffset = 0;
	bool odd_row = true;
	bool odd_col = true;
	bool green = true;
	for (int i = 0; i < sizeX; ++i) {
		if (i % 20 == 0)
			odd_row = !odd_row;

		odd_col = true;
		for (int j = 0; j < sizeY; ++j) {
			if (j % 20 == 0)
				odd_col = !odd_col;

			green = (odd_row && odd_col) || (!odd_row && !odd_col);

			MeshT::Color cp;
			if (green)
				cp = MeshT::Color(0, 255, 0, 255);
			else
				cp = MeshT::Color(255, 0, 255, 255);

			for (int m = 0; m < 3; ++m)
				texBuf[elementOffset++] = cp[m];
		}
	}

	checkerBoardTex_.bind();
	checkerBoardTex_.parameter(GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	checkerBoardTex_.parameter(GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	checkerBoardTex_.setData(0, GL_RGB32F, sizeX, sizeY, GL_RGB, GL_FLOAT, &texBuf[0]);
}

template <class MeshT>
void BezierTriangleMeshNode<MeshT>::createCheckerBoardImage()
{
	QImage b(256, 256, QImage::Format_ARGB32);

	bool odd_row = true;
	bool odd_col = true;
	bool green = true;
	for (int i = 0; i < 256; ++i) {
		if (i % 20 == 0)
			odd_row = !odd_row;

		odd_col = true;
		for (int j = 0; j < 256; ++j) {
			if (j % 20 == 0)
				odd_col = !odd_col;

			green = (odd_row && odd_col) || (!odd_row && !odd_col);

			if (green)
				b.setPixel (i, j, qRgba(0, 255, 0, 255));
			else
				b.setPixel (i, j, qRgba(255, 0, 255, 255));
		}
	}

    b.save("checkerboardTexture.png", "PNG");

	checkerboard_image_ = QGLWidget::convertToGLFormat(b);

	// bind texture
	ACG::GLState::bindTexture(GL_TEXTURE_2D, checkerboard_idx_);
	glTexImage2D(GL_TEXTURE_2D,
		0, GL_RGBA, pick_texture_image_.width(), checkerboard_image_.height(),
		0, GL_RGBA, GL_UNSIGNED_BYTE, checkerboard_image_.bits());
}

//=============================================================================
} // namespace SceneGraph
} // namespace ACG
//=============================================================================
