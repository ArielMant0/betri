#pragma once

#include <QObject>
#include <QAction>

#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/RenderInterface.hh>

#include <ACG/GL/IRenderer.hh>
#include <ACG/GL/FBO.hh>

#include <ACG/GL/globjects.hh>

class RaytracingRenderer : public QObject, BaseInterface, RenderInterface, ACG::IRenderer
{
	Q_OBJECT
		Q_INTERFACES(BaseInterface)
		Q_INTERFACES(RenderInterface)

		Q_PLUGIN_METADATA(IID "org.OpenFlipper.Plugins.Plugin-Render-Raytracing")

public:
	RaytracingRenderer();
	~RaytracingRenderer();

	QString name()
	{
		return (QString("Raytracing Renderer Plugin"));
	};
	QString description()
	{
		return (QString(tr("Render with ratracing")));
	};

public slots:
	QString version()
	{
		return QString("1.0");
	};
	QString renderObjectsInfo(bool _outputShaderInfo);

	QAction* optionsAction();

private slots:

	//BaseInterface
	void initializePlugin();
	void exit();

	// RenderInterface
	void render(ACG::GLState* _glState, Viewer::ViewerProperties& _properties);
	QString rendererName()
	{
		return QString("Raytracing_Renderer");
	}
	void supportedDrawModes(ACG::SceneGraph::DrawModes::DrawMode& _mode)
	{
		_mode = ACG::SceneGraph::DrawModes::DEFAULT;
	}

	QString checkOpenGL();


	void actionDialog(bool);
	void paletteSizeChanged(int);
	void outlineColorChanged(QColor);

private:

	void setupObjectTextures(); //TODO

	void loadShader();

	/// outline shader: multiply scene color with edge factor derived from edges in depth buffer
	GLSL::Program* progOutline_;

	/// size of cel shading palette
	float paletteSize_;

	/// outline color
	ACG::Vec3f outlineCol_;

	/// Collection of fbos for each viewport
	struct ViewerResources
	{
		ViewerResources() : scene_(0)
		{}
		~ViewerResources()
		{
			delete scene_;
		}

		void resize(int _newWidth, int _newHeight);

		ACG::FBO* scene_;
	};

	/**
	* Stores fbo resources for each viewport.
	* Mapping: viewerID -> ViewerResources
	*/
	std::map<int, ViewerResources> viewerRes_;

	ACG::Texture2D controlPointTex_; // TODO make it a vector
	void addTextureToVector(std::vector<float> buffer, const size_t width, const size_t height, int type = 0); // TODO
	std::vector<ACG::Texture2D> objectTex_;
};
