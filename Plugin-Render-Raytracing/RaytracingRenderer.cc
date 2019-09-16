#include <ACG/GL/acg_glew.hh>
#include "RaytracingRenderer.hh"

#include <OpenFlipper/common/GlobalOptions.hh>
#include <OpenFlipper/BasePlugin/PluginFunctions.hh>
#include <ACG/GL/ShaderCache.hh>
#include <ACG/GL/ScreenQuad.hh>
#include <ACG/GL/GLError.hh>

// =================================================

#define CELSHADING_INCLUDE_FILE "RaytracingRenderer/celshading.glsl"
#define OUTLINE_VERTEXSHADER_FILE "RaytracingRenderer/screenquad.glsl"
#define OUTLINE_FRAGMENTSHADER_FILE "RaytracingRenderer/outline.glsl"

#define OBJECT_TYPE_COUNT 3

// Raytracer

/*
setupTextures()
{
	// loop all Render Objects and create a texture for them
	for (int i = 0; i < getNumRenderObjects(); ++i)
		createTexture(getRenderObject(i));
}

setupFragmentShader()
{
	setupTextures();
	// Add more or less Buffer objects
	recompileShader();
}

setupBuffers()
{
	// loop all Render Objects and create a buffer for them
	for (int i = 0; i < getNumRenderObjects(); ++i)
		createBuffer(getRenderObject(i));
}

setupComputeShader()
{
	setupBuffers();
	// Add more or less Buffer objects
	recompileShader();
}
*/

class CelShadingModifier : public ACG::ShaderModifier
{
public:

	void modifyVertexIO(ACG::ShaderGenerator* _shader)
	{
		// include cel lighting functions defined in CELSHADING_INCLUDE_FILE
		QString includeCelShading = ACG::ShaderProgGenerator::getShaderDir() + QDir::separator() + QString(CELSHADING_INCLUDE_FILE);
		_shader->addIncludeFile(includeCelShading);

		// add shader constant that defines the number of different intensity levels used in lighting
		_shader->addUniform("float g_celPaletteSize", "//number of palettes/intensity levels for cel shading");
	}

	void modifyFragmentIO(ACG::ShaderGenerator* _shader)
	{
		// include cel lighting functions defined in CELSHADING_INCLUDE_FILE
		QString includeCelShading = ACG::ShaderProgGenerator::getShaderDir() + QDir::separator() + QString(CELSHADING_INCLUDE_FILE);
		_shader->addIncludeFile(includeCelShading);

		// Note: We include the cel lighting functions in both shader stages
		// because the ShaderGenerator may call modifyLightingCode() for either a vertex or fragment shader.
		// It is not yet known in which stage the lighting is performed.


		// Additionally write the depth of each fragment to a secondary render-target.
		// This depth texture is used in a post-processing outlining step.
		_shader->addOutput("float outDepth");
		_shader->addUniform("float g_celPaletteSize", "//number of palettes/intensity levels for cel shading");
	}


	void modifyFragmentEndCode(QStringList* _code)
	{
		_code->push_back("outDepth = gl_FragCoord.z;"); // write depth to secondary render texture
	}

	// modifier replaces default lighting with cel lighting
	bool replaceDefaultLightingCode()
	{
		return true;
	}

	void modifyLightingCode(QStringList* _code, int _lightId, ACG::ShaderGenLightType _lightType)
	{
		// use cel shading functions instead of default lighting:

		QString buf;

		switch (_lightType) {
			case ACG::SG_LIGHT_DIRECTIONAL:
				buf.sprintf("sg_cColor.xyz += LitDirLight_Cel(sg_vPosVS.xyz, sg_vNormalVS, g_vLightDir_%d,  g_cLightAmbient_%d,  g_cLightDiffuse_%d,  g_cLightSpecular_%d, g_celPaletteSize);", _lightId, _lightId, _lightId, _lightId);
				break;

			case ACG::SG_LIGHT_POINT:
				buf.sprintf("sg_cColor.xyz += LitPointLight_Cel(sg_vPosVS.xyz, sg_vNormalVS,  g_vLightPos_%d,  g_cLightAmbient_%d,  g_cLightDiffuse_%d,  g_cLightSpecular_%d,  g_vLightAtten_%d, g_celPaletteSize);", _lightId, _lightId, _lightId, _lightId, _lightId);
				break;

			case ACG::SG_LIGHT_SPOT:
				buf.sprintf("sg_cColor.xyz += LitSpotLight_Cel(sg_vPosVS.xyz,  sg_vNormalVS,  g_vLightPos_%d,  g_vLightDir_%d,  g_cLightAmbient_%d,  g_cLightDiffuse_%d,  g_cLightSpecular_%d,  g_vLightAtten_%d,  g_vLightAngleExp_%d, g_celPaletteSize);", _lightId, _lightId, _lightId, _lightId, _lightId, _lightId, _lightId);
				break;

			default: break;
		}

		_code->push_back(buf);
	}

	static CelShadingModifier instance;
};

CelShadingModifier CelShadingModifier::instance;

// =================================================

RaytracingRenderer::RaytracingRenderer()
	: progOutline_(0), paletteSize_(2.0f), outlineCol_(0.0f, 0.0f, 0.0f)
{
	ACG::ShaderProgGenerator::registerModifier(&CelShadingModifier::instance);
	objectTex_ = std::vector<ACG::Texture2D>();
	objectTex_.reserve(OBJECT_TYPE_COUNT);
}

RaytracingRenderer::~RaytracingRenderer() 
{
}

QString RaytracingRenderer::checkOpenGL()
{
	if (!ACG::openGLVersion(3, 2))
		return QString("Insufficient OpenGL Version! OpenGL 3.2 or higher required");

	// Check extensions
	QString missing("");
	if (!ACG::openGLVersion(1, 5)) // extension is part of opengl spec since version 1.5
	{                            // i recommend removing this check in favor of restricting
								 // to a more modern version of opengl e.g. 2.1 should be minimum
		if (!ACG::checkExtensionSupported("GL_ARB_vertex_buffer_object"))
			missing += "GL_ARB_vertex_buffer_object extension missing\n";
	}
#ifndef __APPLE__
	if (!ACG::openGLVersion(1, 4)) {
		if (!ACG::checkExtensionSupported("GL_ARB_vertex_program"))
			missing += "GL_ARB_vertex_program extension missing\n";
	}
#endif
	return missing;
}

void RaytracingRenderer::initializePlugin() 
{
}

void RaytracingRenderer::exit()
{
	delete progOutline_;
	progOutline_ = 0;

	viewerRes_.clear();
}

QString RaytracingRenderer::renderObjectsInfo(bool _outputShaderInfo)
{
	std::vector<ACG::ShaderModifier*> modifiers;
	modifiers.push_back(&CelShadingModifier::instance);
	return dumpCurrentRenderObjectsToString(&sortedObjects_[0], _outputShaderInfo, &modifiers);
}

void RaytracingRenderer::render(ACG::GLState* _glState, Viewer::ViewerProperties& _properties)
{

	// Cel shading: 
	// - Restriction of the number of lighting intensity levels
	// - in shader: l dot n is quantized based on the number of allowed shading tones.
	// currently a constant sized step function is used to quantize the intensity

	// collect renderobjects + prepare OpenGL state
	prepareRenderingPipeline(_glState, _properties.drawMode(), PluginFunctions::getSceneGraphRootNode());

	// init/update fbos
	ViewerResources* viewRes = &viewerRes_[_properties.viewerId()];
	viewRes->resize(_glState->viewport_width(), _glState->viewport_height());

	glViewport(0, 0, _glState->viewport_width(), _glState->viewport_height());

	if (false /*compute_shader*/) {

		//if (getNumRenderObjects() != object_count)
		//	setupComputeShader();

		// run compute Shader

		// Use Output to render the result texture on the screen
	} else if (true /*fragment_shader*/) {
		//if (getNumRenderObjects() != object_count)
		//if (!controlPointTex_.is_valid())
		if (objectTex_.size() == 0) // TODO
			setupObjectTextures();

		// ----------------------------------------------------------
		// Invoke raytracing fragmentshader

		if (!progOutline_)
			progOutline_ = GLSL::loadProgram(OUTLINE_VERTEXSHADER_FILE, OUTLINE_FRAGMENTSHADER_FILE);

		// restore previous fbo
		restoreInputFbo();


		// enable color/depth write access
		glDepthMask(1);
		glColorMask(1, 1, 1, 1);

		// note: using glDisable(GL_DEPTH_TEST) not only disables depth testing,
		//  but actually discards any write operations to the depth buffer.
		// However, we can provide scene depth for further post-processing. 
		//   -> Enable depth testing with func=always
		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_ALWAYS);
		glDisable(GL_BLEND);
	
		// setup raytracing shader
		progOutline_->use();

		// https://www.openflipper.org/media/Documentation/OpenFlipper-3.0/a00643.html#ae27e2005ea57312f70752b0a2608f506
		// Vertex shader uniforms
		ACG::GLMatrixf mvp =  _glState->modelview();
		progOutline_->setUniform("g_mWVP", mvp);

		progOutline_->setUniform("u_viewportWidth", float(_glState->viewport_width()));
		progOutline_->setUniform("u_viewportHeight", float(_glState->viewport_height()));
		progOutline_->setUniform("u_fovy", float(_glState->fovy()));
		progOutline_->setUniform("g_mWVP", mvp);

		progOutline_->setUniform("u_invproj", _glState->inverse_projection());
		progOutline_->setUniform("u_invmodelview", _glState->inverse_modelview());
		progOutline_->setUniform("u_proj", _glState->projection());
		progOutline_->setUniform("u_modelview", _glState->modelview());
		progOutline_->setUniform("u_near", float(_glState->near_plane()));
		progOutline_->setUniform("u_far", float(_glState->far_plane()));

		//_glState->reset_projection();

		// Framgent shader uniforms
		progOutline_->setUniform("texcoordOffset", ACG::Vec2f(1.0f / float(viewRes->scene_->width()), 1.0f / float(viewRes->scene_->height())));
		//ACG::GLMatrixf mvp = _glState->projection * _glState->modelview;
		//progOutline_->setUniform("g_mWVP", mvp);
		progOutline_->setUniform("g_vCamPos", camPosWS_);
		progOutline_->setUniform("g_vCamPos2", camPosWS_);

		//auto bla = ACG::RenderObject::Texture(controlPointTex_.id(), GL_TEXTURE_2D);

		// TODO
		const std::array<char*, OBJECT_TYPE_COUNT> uniformArray = {
			"cubes", "spheres", "triangles"
		};

		int texIndex = 0;
		for (auto textureUniform : uniformArray) {
			progOutline_->setUniform(textureUniform, int(texIndex));
			glActiveTexture(GL_TEXTURE0 + texIndex);
			glBindTexture(GL_TEXTURE_2D, (objectTex_[texIndex++]).id());
		}

		/*
		TODO old
		progOutline_->setUniform("cubes", int(0));
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, (objectTex_[texIndex++]).id());
		//glBindTexture(GL_TEXTURE_2D, controlPointTex_.id());
		
		progOutline_->setUniform("spheres", int(1));
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, (objectTex_[texIndex++]).id());
		*/

		// draw fullscreenQuad
		// every fragment casts a ray which will do the intersection test
		ACG::ScreenQuad::draw(progOutline_);

		progOutline_->disable();

	} else if (false/*cpu*/) {
		/*
		//_glState->viewing_ray(0, 0, null, null);
		for cpu raytracing
		http://www.openflipper.org/media/Documentation/OpenFlipper-2.1/a00621.html
		viewing_direction (int _x, int _y) const
		get viewing ray through pixel (_x,_y)
		*/
	}

	// reset depth func to opengl default
	glDepthFunc(GL_LESS);

	ACG::glCheckErrors();

	// restore common opengl state
	// log window remains hidden otherwise
	finishRenderingPipeline();
}


void RaytracingRenderer::addTextureToVector(std::vector<float> &buffer, const size_t width, const size_t height, int type)
{
	ACG::Texture2D tex;
	objectTex_.push_back(tex);
	const size_t pos = objectTex_.size() - 1; // TODO

	objectTex_[pos].bind();

	objectTex_[pos].parameter(GL_TEXTURE_MIN_FILTER, GL_NEAREST); // disable filtering
	objectTex_[pos].parameter(GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	objectTex_[pos].parameter(GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	objectTex_[pos].parameter(GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	if (type == 0) // TODO
		objectTex_[pos].setData(0, GL_RGB32F, width, height, GL_RGB, GL_FLOAT, &buffer[0]);
	else
		objectTex_[pos].setData(0, GL_RGBA32F, width, height, GL_RGBA, GL_FLOAT, &buffer[0]);
}


void RaytracingRenderer::setupObjectTextures()
{
//#ifdef GL_VERSION_3_0
	///////////////////////////////////////////////////////////////////////////
	// Add cubes
	///////////////////////////////////////////////////////////////////////////

	const size_t rectangle_count = 2;
	const size_t r_floats = 3;
	const size_t r_points = 2;
	const std::array<std::array<int, r_floats * r_points>, rectangle_count> cubeArray = { {
		{-5.0, -3.0, -5.0, 5.0, -2.9, 5.0},
		{-1.5, -1.5, -1.5, 1.5, 1.5, 1.5}
		//{0, 255, 0, 0, 0, 255},
		//{255, 0, 0, 0, 0, 0}
	} };

	size_t counter = 0;
	std::vector<float> rectangleBuf(rectangle_count * r_floats * r_points);
	for (auto elem : cubeArray) {
		for (auto item : elem) {
			rectangleBuf[counter++] = item;
		}
	}

	addTextureToVector(rectangleBuf, r_points, rectangle_count, 0);

	///////////////////////////////////////////////////////////////////////////
	// Add spheres
	///////////////////////////////////////////////////////////////////////////

	const size_t sphere_count = 2;
	const size_t s_floats = 4;
	const size_t s_points = 2;
	const std::array<std::array<int, s_floats * s_points>, sphere_count> sphereArray = { {
		// Position 3f, Radius 1f, Color 4f
		{0.0, 2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 1.0},
		{10.0, 0.0, 0.0, 1.5, 0.0, 1.0, 1.0, 1.0}
	} };

	counter = 0;
	std::vector<float> sphereBuf(sphere_count * s_floats * s_points);
	for (auto elem : sphereArray) {
		for (auto item : elem) {
			sphereBuf[counter++] = item;
		}
	}

	addTextureToVector(sphereBuf, s_points, sphere_count, 1);

	///////////////////////////////////////////////////////////////////////////
	// Add triangles
	///////////////////////////////////////////////////////////////////////////
	
	const size_t triangle_count = 2;
	const size_t t_floats = 3;
	const size_t t_points = 3 + 1;
	const std::array<std::array<int, t_floats * t_points>, triangle_count> triangleArray = { {
		// Color 3f, Vertex1 3f, Vertex2 3f, Vertex3 3f
		{0.0, 1.0, 0.0,	-2.0, 0.0, 5.0, 2.0, 0.0, 5.0, -2.0, 2.0, 5.0},
		{1.0, 0.0, 0.0,	-2.0, 0.0, -5.0, 2.0, 0.0, -5.0, -2.0, 2.0, -5.0}
		//{0.0, 1.0, 0.0,	0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0},
		//{1.0, 0.0, 0.0,	0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0}
	} };

	counter = 0;
	std::vector<float> triangleBuf(triangle_count * t_floats * t_points);
	for (auto elem : triangleArray) {
		for (auto item : elem) {
			triangleBuf[counter++] = item;
		}
	}

	addTextureToVector(triangleBuf, t_points, triangle_count);
	
//#endif
}


QAction* RaytracingRenderer::optionsAction()
{
	QAction * action = new QAction("Raytracing Renderer Options", this);

	connect(action, SIGNAL(triggered(bool)), this, SLOT(actionDialog(bool)));

	return action;
}

void RaytracingRenderer::paletteSizeChanged(int _val)
{
	paletteSize_ = float(_val) / 100.0f;
}

void RaytracingRenderer::outlineColorChanged(QColor _col)
{
	outlineCol_[0] = _col.redF();
	outlineCol_[1] = _col.greenF();
	outlineCol_[2] = _col.blueF();
}

void RaytracingRenderer::ViewerResources::resize(int _newWidth, int _newHeight)
{
	if (!_newHeight || !_newWidth)
		return;

	if (!scene_) {
		// scene fbo with 2 color attachments + depth buffer
		//  attachment0: scene color
		//  attachment1: scene depth
		scene_ = new ACG::FBO();
		scene_->init();
		scene_->attachTexture2D(GL_COLOR_ATTACHMENT0, _newWidth, _newHeight, GL_RGBA, GL_RGBA);
		scene_->attachTexture2D(GL_COLOR_ATTACHMENT1, _newWidth, _newHeight, GL_R32F, GL_RED);
		scene_->attachTexture2DDepth(_newWidth, _newHeight);
	}

	if (scene_->height() == _newHeight && scene_->width() == _newWidth)
		return;

	scene_->resize(_newWidth, _newHeight);
}
