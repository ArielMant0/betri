/**
 * \file Benchmarker.hh
 *
 */

#pragma once
///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <ACG/GL/gl.hh>
#include <ACG/GL/acg_glew.hh>
#include <ACG/GL/globjects.hh>

#include <ACG/Config/ACGDefines.hh>

///////////////////////////////////////////////////////////////////////////////
// Defines
///////////////////////////////////////////////////////////////////////////////
#define RENDERMODE_NUM 7
#define QUERY_COUNT 100

///////////////////////////////////////////////////////////////////////////////
// Namespaces
///////////////////////////////////////////////////////////////////////////////
namespace ACG
{

///////////////////////////////////////////////////////////////////////////////
// CLASS DEFINITION
///////////////////////////////////////////////////////////////////////////////
class ACGDLLEXPORT Benchmarker
{
public:
	enum RENDER_MODE : int
	{
		CPU = 1,
		GPU = 2,
		RAYTETRA = 4,
		RAYAABB = 8,
		RAYPRISM = 16,
		RAYCHULL = 32,
		RAYBILLB = 64,
	};

	static Benchmarker* instance();

	void startFrame();
	void endFrame();

	void active(bool activate);
	bool active() const;
	bool update();

	void renderMode(int rmode);
	int renderMode() const;
	int bVolume() const;

private:
	Benchmarker() :
		current_query_(0), active_(false), updateBuffers_(false),
		renderModes_(0), renderModesDump_(0),
		activeRMode_(0), activeBVol_(0), shifted_(0)
	{
		for (int i = 0; i < RENDERMODE_NUM; i++)
			glGenQueries(QUERY_COUNT, queries_[i].data());

		// TODO
		// https://www.khronos.org/opengl/wiki/OpenGL_Context#Context_information_queries
		// https://stackoverflow.com/questions/42245870/how-to-get-the-graphics-card-model-name-in-opengl-or-win32
		const GLubyte* renderer = glGetString(GL_RENDERER);
		const GLubyte* vendor = glGetString(GL_VENDOR);

		std::cerr << vendor << std::endl;
		std::cerr << renderer << std::endl;
	}

	void advanceRenderMode();
	
	void dumpToFile();

///////////////////////////////////////////////////////////////////////////////
// Private Membervariables
///////////////////////////////////////////////////////////////////////////////
private:
	//GLuint queries_[QUERY_COUNT];
	std::array<std::array<GLuint, QUERY_COUNT>, RENDERMODE_NUM> queries_;
	int current_query_;

	bool active_;
	bool updateBuffers_;
	int renderModes_;
	int renderModesDump_;
	int activeRMode_;
	int activeBVol_;
	int shifted_;
	
	std::string fileName_;
};

} // namespace vis
