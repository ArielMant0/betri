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

class CelShadingModifier : public ACG::ShaderModifier{
public:

  void modifyVertexIO( ACG::ShaderGenerator* _shader )  {
    // include cel lighting functions defined in CELSHADING_INCLUDE_FILE
    QString includeCelShading = ACG::ShaderProgGenerator::getShaderDir() + QDir::separator() + QString(CELSHADING_INCLUDE_FILE);
    _shader->addIncludeFile(includeCelShading);

    // add shader constant that defines the number of different intensity levels used in lighting
    _shader->addUniform("float g_celPaletteSize", "//number of palettes/intensity levels for cel shading");
  }

  void modifyFragmentIO( ACG::ShaderGenerator* _shader )  {
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


  void modifyFragmentEndCode(QStringList* _code)  {
    _code->push_back("outDepth = gl_FragCoord.z;"); // write depth to secondary render texture
  }

  // modifier replaces default lighting with cel lighting
  bool replaceDefaultLightingCode() {return true;}

  void modifyLightingCode(QStringList* _code, int _lightId, ACG::ShaderGenLightType _lightType)  {
    // use cel shading functions instead of default lighting:

    QString buf;

    switch (_lightType)    {
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
}


RaytracingRenderer::~RaytracingRenderer() {
}

QString RaytracingRenderer::checkOpenGL() {
  if ( !ACG::openGLVersion(3, 2) )
    return QString("Insufficient OpenGL Version! OpenGL 3.2 or higher required");

  // Check extensions
  QString missing("");
  if(!ACG::openGLVersion(1,5)) // extension is part of opengl spec since version 1.5
  {                            // i recommend removing this check in favor of restricting
                               // to a more modern version of opengl e.g. 2.1 should be minimum
    if ( !ACG::checkExtensionSupported("GL_ARB_vertex_buffer_object") )
        missing += "GL_ARB_vertex_buffer_object extension missing\n";
  }
#ifndef __APPLE__
  if(!ACG::openGLVersion(1,4))
  {
    if ( !ACG::checkExtensionSupported("GL_ARB_vertex_program") )
        missing += "GL_ARB_vertex_program extension missing\n";
  }
#endif
  return missing;
}

void RaytracingRenderer::initializePlugin() {
}

void RaytracingRenderer::exit() {
  delete progOutline_;
  progOutline_ = 0;

  viewerRes_.clear();
}

QString RaytracingRenderer::renderObjectsInfo(bool _outputShaderInfo) {
  std::vector<ACG::ShaderModifier*> modifiers;
  modifiers.push_back(&CelShadingModifier::instance);
  return dumpCurrentRenderObjectsToString(&sortedObjects_[0], _outputShaderInfo, &modifiers);
}

void RaytracingRenderer::render(ACG::GLState* _glState, Viewer::ViewerProperties& _properties) {

  // Cel shading: 
  // - Restriction of the number of lighting intensity levels
  // - in shader: l dot n is quantized based on the number of allowed shading tones.
  // currently a constant sized step function is used to quantize the intensity

  // collect renderobjects + prepare OpenGL state
  prepareRenderingPipeline(_glState, _properties.drawMode(), PluginFunctions::getSceneGraphRootNode());

  // init/update fbos
  ViewerResources* viewRes = &viewerRes_[_properties.viewerId()];
  viewRes->resize(_glState->viewport_width(), _glState->viewport_height());

  // --------------------------------------------------
  // 1st pass: draw scene to intermediate fbo

  // enable color/depth write access
  glDepthMask(1);
  glColorMask(1,1,1,1);

  // bind fbo for scene + depth tex
  viewRes->scene_->bind();

  glViewport(0, 0, _glState->viewport_width(), _glState->viewport_height());

  // clear depth texture
  glDrawBuffer(GL_COLOR_ATTACHMENT1);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // clear scene color
  glDrawBuffer(GL_COLOR_ATTACHMENT0);
  ACG::Vec4f clearColor = _properties.backgroundColor();
  glClearColor(clearColor[0], clearColor[1], clearColor[2], 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // attachment0: scene colors
  // attachment1: scene depth
  GLenum colorDepthTarget[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
  glDrawBuffers(2, colorDepthTarget);

  // render every object
  for (int i = 0; i < getNumRenderObjects(); ++i) {

    // Take original shader and modify the output to take only the normal as the color
    GLSL::Program* prog = ACG::ShaderCache::getInstance()->getProgram(&sortedObjects_[i]->shaderDesc, CelShadingModifier::instance);

    int bRelink = 0;
    if (prog->getFragDataLocation("outFragment") != 0) {
      prog->bindFragDataLocation(0, "outFragment");
      bRelink++;
    }
    if (prog->getFragDataLocation("outDepth") != 1) {
      prog->bindFragDataLocation(1, "outDepth");
      bRelink++;
    }
    if (bRelink)
      prog->link();

    prog->use();
    prog->setUniform("g_celPaletteSize", paletteSize_);

    renderObject(sortedObjects_[i], prog);
  }

  viewRes->scene_->unbind();

  // ----------------------------------------------------------
  // 2nd pass: compose final image with outlining as  scene color * edge factor

  if (!progOutline_)
    progOutline_ = GLSL::loadProgram(OUTLINE_VERTEXSHADER_FILE, OUTLINE_FRAGMENTSHADER_FILE);

  // restore previous fbo
  restoreInputFbo();


  // enable color/depth write access
  glDepthMask(1);
  glColorMask(1,1,1,1);

  // note: using glDisable(GL_DEPTH_TEST) not only disables depth testing,
  //  but actually discards any write operations to the depth buffer.
  // However, we can provide scene depth for further post-processing. 
  //   -> Enable depth testing with func=always
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_ALWAYS);
  glDisable(GL_BLEND);

  // setup composition shader
  progOutline_->use();
  progOutline_->setUniform("samplerScene", 0);
  progOutline_->setUniform("samplerDepth", 1);
  progOutline_->setUniform("texcoordOffset", ACG::Vec2f(1.0f / float(viewRes->scene_->width()), 1.0f / float(viewRes->scene_->height()) ));
  progOutline_->setUniform("clipPlanes", ACG::Vec2f(_glState->near_plane(), _glState->far_plane()));
  progOutline_->setUniform("outlineColor", outlineCol_);

  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, viewRes->scene_->getAttachment(GL_COLOR_ATTACHMENT1));

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, viewRes->scene_->getAttachment(GL_COLOR_ATTACHMENT0));


  ACG::ScreenQuad::draw(progOutline_);

  progOutline_->disable();

  // reset depth func to opengl default
  glDepthFunc(GL_LESS);
  
  ACG::glCheckErrors();

  // restore common opengl state
  // log window remains hidden otherwise
  finishRenderingPipeline();
}


QAction* RaytracingRenderer::optionsAction()
{
  QAction * action = new QAction("Raytracing Renderer Options" , this );

  connect(action,SIGNAL(triggered( bool )),this,SLOT(actionDialog( bool )));

  return action;
}

void RaytracingRenderer::paletteSizeChanged( int _val ) {
  paletteSize_ = float(_val) / 100.0f;
}

void RaytracingRenderer::outlineColorChanged( QColor _col ) {
  outlineCol_[0] = _col.redF();
  outlineCol_[1] = _col.greenF();
  outlineCol_[2] = _col.blueF();
}

void RaytracingRenderer::ViewerResources::resize( int _newWidth, int _newHeight ) {
  if (!_newHeight || !_newWidth) return;


  if (!scene_)  {
    // scene fbo with 2 color attachments + depth buffer
    //  attachment0: scene color
    //  attachment1: scene depth
    scene_ = new ACG::FBO();
    scene_->init();
    scene_->attachTexture2D(GL_COLOR_ATTACHMENT0, _newWidth, _newHeight, GL_RGBA, GL_RGBA);
    scene_->attachTexture2D(GL_COLOR_ATTACHMENT1, _newWidth, _newHeight, GL_R32F, GL_RED);
    scene_->attachTexture2DDepth(_newWidth, _newHeight);
  }

  if (scene_->height() == _newHeight &&
    scene_->width()  == _newWidth)
    return;

  scene_->resize(_newWidth, _newHeight);
}




