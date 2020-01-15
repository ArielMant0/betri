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

	/**
	 * Get a benchmarker
	 */
	static Benchmarker* instance();

	/**
	 * Start measuring. This is using GL queries, possible options
	 * are time and samples.
	 * https://www.khronos.org/opengl/wiki/Query_Object
	 */
	void startFrame();

	/**
	 * Should be called, to collect the querie data given by the GPU
	 * calls glEndQuery and saves the data in an array
	 */
	void endFrame();

///////////////////////////////////////////////////////////////////////////////
// Mode getter/setter
///////////////////////////////////////////////////////////////////////////////
	/**
	 * Activate the benchmarker
	 * @param activate
	 */
	void active(bool activate);
	/**
	 * Return true if the benchmarker is active.
	 * Can be used to suspend other rendering, to not interfere with the results.
	 * @returns active
	 */
	bool active() const;
	/**
	 * Return if one rendermode finished and if the buffer should be updated.
	 * @return update
	 */
	bool update();

	/**
	 * Set the type to test
	 * @param testType
	 */
	void testType(TEST_TYPE testType);
	/**
	 * Set the type to test against
	 * @param againstType
	 */
	void againstType(AGAINST_TYPE againstType);
	/**
	 * Shall the results be averaged?
	 * @param state
	 */
	void average(bool state);
	/**
	 * Shall the results be appended to the given file path?
	 * @param state
	 */
	void append(bool state);

	/**
	 * Defines which rendermodes are iteratied, an int should be given where each bit
	 * defines whether a rendermode is run
	 * @param rmode
	 */
	void renderMode(int rmode);
	/**
	 * Return which rendermode is at the moment active.
	 * @return rendermode
	 */
	int renderMode() const;
	/**
	 * Return which boundingvolume is at the moment active if any.
	 * @return bvolume
	 */
	int bVolume() const;

	/**
	 * Set the attributes of the mesh, such that it can be saved in the filename
	 * @param name
	 * @param triangleCount
	 * @param degree
	 */
	void meshInfo(std::string name, int triangleCount, int degree);

///////////////////////////////////////////////////////////////////////////////
// File string setter
///////////////////////////////////////////////////////////////////////////////
	/**
	 * Set the file path
	 * @param path
	 */
	void filePath(std::string path);
	/**
	 * Set the file name
	 * @param name
	 */
	void fileName(std::string name);
	/**
	 * Set the file type
	 * @param type
	 */
	void fileType(std::string type);

private:
	/**
	 * Constuctor
	 * Creates the initial Benchmarker object and sets up all variables.
	 * Also prepares the query array
	 */
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

	/**
	 * Advances render mode. Can be done when other one finished.
	 */
	void advanceRenderMode();

	/**
	 * Use functions to get the cpu name
	 * @return cpuname
	 */
	std::string getCPUName();
	/**
	 * Construct filename from cpu and gpu, this is done by determine which system has
	 * these components
	 * @return filename addition
	 */
	std::string generateFileName(std::string cpuName, std::string gpuName);
	/**
	 * Returns a string for the type of the first column. Corresponds directly to the
	 * againsttypes
	 * @param which one to use
	 * @returns the against type
	 */
	std::string columnOne(int i);
	/**
	 * Print all information and data into the specified file
	 */
	void dumpToFile();

///////////////////////////////////////////////////////////////////////////////
// Private Membervariables
///////////////////////////////////////////////////////////////////////////////
private:
	//GLuint queries_[QUERY_COUNT];
	std::array<std::array<GLuint, QUERY_COUNT>, RENDERMODE_NUM> queries_;
	int current_query_;

	//! Is the benchmarkes acitve
	bool active_;
	//! Should the buffer be updated
	bool updateBuffers_;
	//! Should the result be averages
	bool average_;
	//! Should the resuls be appended
	bool append_;

	//! For what should be tested
	TEST_TYPE testType_;
	//! Against what should be tested
	int againstType_;

	//! Which rendermodes to run (is modified)
	int renderModes_;
	//! Not modified rendermodes version
	int renderModesDump_;
	//! Activated render mode
	int activeRMode_;
	//! Activated bounding volume
	int activeBVol_;
	//! How much the rendermode is shifted
	int shifted_;

	//! Filepath
	std::string path_;
	//! Filename
	std::string name_;
	//! Filetype
	std::string type_;

	//! How the mesh is called
	std::string meshName_;
	//! How many triangles it has
	int triangleCount_;
	//! What is the degree
	int degree_;
};

} // namespace vis
