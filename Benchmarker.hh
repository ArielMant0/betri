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
#define RENDERMODE_NUM 8
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
		RAYMESH = 64,
		RAYBILLB = 128
	};

	static Benchmarker* instance();

	void startFrame();
	void endFrame();

	void active(bool activate);
	bool active() const;
	bool update();

	void occlQuery(bool state);
	void average(bool state);

	void renderMode(int rmode);
	int renderMode() const;
	int bVolume() const;

private:
	Benchmarker() :
		current_query_(0), active_(false), updateBuffers_(false),
		renderModes_(0), renderModesDump_(0),
		activeRMode_(0), activeBVol_(0), shifted_(0),
		average_(false), occlQuery_(false)
	{
		for (int i = 0; i < RENDERMODE_NUM; i++)
			glGenQueries(QUERY_COUNT, queries_[i].data());
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
	bool average_;
	bool occlQuery_;

	int renderModes_;
	int renderModesDump_;
	int activeRMode_;
	int activeBVol_;
	int shifted_;
};

} // namespace vis
