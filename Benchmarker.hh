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

	enum TEST_TYPE : int
	{
		FTIME = 0,
		OCCL = 1
	};

	enum AGAINST_TYPE : int
	{
		FRAME = 0,
		TC = 1,
		DEGREE = 2,
		GENUS = 3
	};

	static Benchmarker* instance();

	void startFrame();
	void endFrame();

///////////////////////////////////////////////////////////////////////////////
// Mode getter/setter
///////////////////////////////////////////////////////////////////////////////
	void active(bool activate);
	bool active() const;
	bool update();

	void testType(TEST_TYPE testType);
	void againstType(AGAINST_TYPE againstType);
	void average(bool state);
	void append(bool state);

	void renderMode(int rmode);
	int renderMode() const;
	int bVolume() const;

	void meshInfo(std::string name, int triangleCount, int degree);

///////////////////////////////////////////////////////////////////////////////
// File string setter
///////////////////////////////////////////////////////////////////////////////
	void filePath(std::string path);
	void fileName(std::string name);
	void fileType(std::string type);

private:
	Benchmarker() :
		active_(false), current_query_(0), updateBuffers_(false),
		renderModes_(0), renderModesDump_(0),
		activeRMode_(0), activeBVol_(0), shifted_(0),
		average_(false), append_(false),
		testType_(TEST_TYPE::FTIME), againstType_(AGAINST_TYPE::FRAME)
	{
		for (int i = 0; i < RENDERMODE_NUM; i++)
			glGenQueries(QUERY_COUNT, queries_[i].data());
	}

	void advanceRenderMode();

	std::string getCPUName();
	std::string generateFileName(std::string cpuName, std::string gpuName);
	std::string columnOne(int i);
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
	bool append_;

	TEST_TYPE testType_;
	int againstType_;

	int renderModes_;
	int renderModesDump_;
	int activeRMode_;
	int activeBVol_;
	int shifted_;

	std::string path_;
	std::string name_;
	std::string type_;

	std::string meshName_;
	int triangleCount_;
	int degree_;
};

} // namespace vis
