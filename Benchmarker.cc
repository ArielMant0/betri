///////////////////////////////////////////////////////////////////////////////
// Defines
///////////////////////////////////////////////////////////////////////////////
#if defined(_WIN32)
#define PLATFORM_NAME windows
#elif defined(_WIN64)
#define PLATFORM_NAME windows
#elif defined(__CYGWIN__) && !defined(_WIN32)
#define PLATFORM_NAME windows
#endif

///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////
#include "Benchmarker.hh"
#include <cctype>
#if PLATFORM_NAME == windows
#include <intrin.h>
#endif
#include <experimental/filesystem>

static ACG::Benchmarker *bench = nullptr;


///////////////////////////////////////////////////////////////////////////////
// Namespaces
///////////////////////////////////////////////////////////////////////////////
namespace ACG
{

Benchmarker* Benchmarker::instance()
{
	if (bench == nullptr) {
		bench = new Benchmarker();
	}

	return bench;
}

void Benchmarker::startFrame()
{
	if (active()) {
		// start query
		switch (testType_) {
			case TEST_TYPE::FTIME: glBeginQuery(
				GL_TIME_ELAPSED, queries_[shifted_ - 1][current_query_]); 
				break;
			case TEST_TYPE::OCCL: glBeginQuery(
				GL_SAMPLES_PASSED, queries_[shifted_ - 1][current_query_]); 
				break;
			default:
				std::cerr << __FUNCTION__ << " nothing done" << std::endl; // TODO
		}
	}
}

void Benchmarker::endFrame()
{
	if (active()) {
		switch (testType_) {
			case TEST_TYPE::FTIME: glEndQuery(GL_TIME_ELAPSED); break;
			case TEST_TYPE::OCCL: glEndQuery(GL_SAMPLES_PASSED); break;
			default:
				std::cerr << __FUNCTION__ << " nothing done" << std::endl; 
		}

		// advance query counter
		if (current_query_ + 1 == QUERY_COUNT) {
			current_query_ = 0;
			advanceRenderMode();
		} else {
			current_query_++;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// Mode getter/setter
///////////////////////////////////////////////////////////////////////////////
void Benchmarker::active(bool activate)
{
	active_ = activate;
}

bool Benchmarker::active() const
{
	return active_;
}

bool Benchmarker::update()
{
	bool tmp = updateBuffers_;
	updateBuffers_ = false;
	return tmp;
}

void Benchmarker::testType(TEST_TYPE testType)
{
	testType_ = testType;
}

void Benchmarker::againstType(AGAINST_TYPE againstType)
{
	againstType_ = againstType;
}

void Benchmarker::average(bool state)
{
	average_ = state;
}

void Benchmarker::append(bool state)
{
	append_ = state;
}

void Benchmarker::renderMode(int rmode)
{
	renderModes_ = rmode;
	renderModesDump_ = rmode;
	advanceRenderMode();
}

int Benchmarker::renderMode() const
{
	return activeRMode_;
}

int Benchmarker::bVolume() const
{
	return activeBVol_;
}

void Benchmarker::meshInfo(std::string name, int triangleCount, int degree)
{
	meshName_ = name;
	triangleCount_ = triangleCount;
	degree_ = degree;
}

///////////////////////////////////////////////////////////////////////////////
// File string setter
///////////////////////////////////////////////////////////////////////////////
void Benchmarker::filePath(std::string path)
{
	path_ = path;
}

void Benchmarker::fileName(std::string name)
{
	name_ = name;
}

void Benchmarker::fileType(std::string type)
{
	type_ = type;
}

///////////////////////////////////////////////////////////////////////////////
// Logic
///////////////////////////////////////////////////////////////////////////////
void Benchmarker::advanceRenderMode()
{
	updateBuffers_ = true;

	// If everthing is finished deactivate the benchmarker and print the file
	// Clear the queries
	if (renderModes_ == 0) {
		active(false);
		dumpToFile();
		activeRMode_ = 0;
		activeBVol_ = 0;
		shifted_ = 0;
		for (int i = 0; i < RENDERMODE_NUM; i++)
			glGenQueries(QUERY_COUNT, queries_[i].data());
		return;
	}

	// If not finished shift the rendermode
	for (; !(renderModes_ & 1); shifted_++) {
		renderModes_ = renderModes_ >> 1;
	}

	// determine what should be rendered based on the mode
	switch (shifted_)
	{
		case 0: activeRMode_ = 1; break;
		case 1: activeRMode_ = 2; break;
		case 2: activeRMode_ = 3; activeBVol_ = 0; break;
		case 3: activeRMode_ = 3; activeBVol_ = 1; break;
		case 4: activeRMode_ = 3; activeBVol_ = 2; break;
		case 5: activeRMode_ = 3; activeBVol_ = 3; break;
		case 6: activeRMode_ = 3; activeBVol_ = 4; break;
		case 7: activeRMode_ = 3; activeBVol_ = 5; break;
		default: break;
	}
	shifted_++;
	renderModes_ = renderModes_ >> 1;
	assert(shifted_ < RENDERMODE_NUM + 1);
}

// https://weseetips.wordpress.com/tag/c-get-cpu-name/
std::string Benchmarker::getCPUName()
{
#if PLATFORM_NAME == windows
	// Get extended ids.
	int CPUInfo[4] = { -1 };
	__cpuid(CPUInfo, 0x80000000);
	unsigned int nExIds = CPUInfo[0];

	// Get the information associated with each extended ID.
	char CPUBrandString[0x40] = { 0 };
	for (unsigned int i = 0x80000000; i <= nExIds; ++i) {
		__cpuid(CPUInfo, i);

		// Interpret CPU brand string and cache information.
		if (i == 0x80000002) {
			memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
		} else if (i == 0x80000003) {
			memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
		} else if (i == 0x80000004) {
			memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
		}
	}

	return std::string(CPUBrandString);
#elif
	return std::string("");
#endif
}

std::string Benchmarker::generateFileName(
	std::string cpuName, std::string gpuName
)
{
	// Compare the strings to the prepared systems, if it none of 
	// the no system informations are added
	std::string sys = "";
	if (cpuName.compare("AMD Ryzen 3 1300X Quad-Core Processor") == 0 &&
		gpuName.compare("GeForce GTX 1050 Ti/PCIe/SSE2") == 0
	) {
		sys = "Sys1";
	} else if (
		cpuName.compare("Intel(R) Core(TM) i7-7700HQ CPU @ 2.80GHz") == 0 &&
		gpuName.compare("Intel(R) HD Graphics 630") == 0
	) {
		sys = "Sys2";
	} else if (
		cpuName.compare("Intel(R) Core(TM) i7-7700HQ CPU @ 2.80GHz") == 0 &&
		gpuName.compare("GeForce GTX 1050 Ti/PCIe/SSE2") == 0
	) {
		sys = "Sys3";
	} else if (
		cpuName.compare("Intel(R) Core(TM) i7-3630HQ CPU @ 2.40GHz") == 0 &&
		gpuName.compare("Intel(R) HD Graphics 4000") == 0
	) {
		sys = "Sys4";
	} else if (
		cpuName.compare("Intel(R) Core(TM) i7-3630HQ CPU @ 2.40GHz") == 0 &&
		gpuName.compare("GeForce 660M PCIe/SSE2") == 0
	) {
		sys = "Sys5";
	}

	// Chose prefix
	int cgr = RENDER_MODE::CPU | RENDER_MODE::GPU | RENDER_MODE::RAYPRISM;
	int ray = RENDER_MODE::RAYAABB | RENDER_MODE::RAYPRISM;
	std::string prefix = "";
	if ((renderModesDump_ & cgr) == cgr) {
		prefix = "CGR";
	} else if ((renderModesDump_ & ray) == ray) {
		prefix = "Ray";
	}

	// Choose the basetype
	std::string baseType;
	switch (testType_) {
		case TEST_TYPE::FTIME: baseType = "Ftime"; break;
		case TEST_TYPE::OCCL: baseType = "Occl"; break;
		default: baseType = "";
	}

	// Chose the type to compare against
	std::string againstType;
	switch (againstType_) {
		case 0: 
			againstType = "Frame" + meshName_ + 
				"D" + std::to_string(degree_) +
				"Tc" + std::to_string(triangleCount_);
			break;
		case 1: againstType = "Tc" + meshName_ + "D" + std::to_string(degree_); 
			break;
		case 2: againstType = "Degree" + meshName_ + 
			"Tc" + std::to_string(triangleCount_); break;
		case 3: againstType = "Genus" + 
			("Tc" + std::to_string(triangleCount_)) +
			"D" + std::to_string(degree_); break;
		default: againstType = "";
	}

	// Combine
	return prefix + baseType + "Vs" + againstType + sys;
}

std::string Benchmarker::columnOne(int i)
{
	switch (againstType_) {
		case AGAINST_TYPE::FRAME: return std::to_string(i + 1);
		case AGAINST_TYPE::TC: return std::to_string(triangleCount_);
		case AGAINST_TYPE::DEGREE: return std::to_string(degree_);
		case AGAINST_TYPE::GENUS: return "1";
		default: return std::to_string(i + 1);
	}
}

void Benchmarker::dumpToFile()
{
	// Get the gpu name
	// https://www.khronos.org/opengl/wiki/OpenGL_Context#Context_information_queries
	// https://stackoverflow.com/questions/42245870/how-to-get-the-graphics-card-model-name-in-opengl-or-win32
	std::string gpuName(reinterpret_cast<const char*>(glGetString(GL_RENDERER)));
	auto cpuName = getCPUName();

	// Construct the path
	std::string complete;
	if (name_.size() > 0)
		complete = path_ + name_ + type_;
	else
		complete = path_ + generateFileName(cpuName, gpuName) + type_;

	// Detect whether the file exists
	// https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c
	auto exists = std::experimental::filesystem::exists(complete);
	std::ofstream fileout(complete, append_ ? std::ofstream::app : std::ofstream::trunc);

	if (!fileout) {
		std::cerr << "Error open fileout!" << std::endl;
		return;
	}

	const std::string modes[RENDERMODE_NUM] = {
		"CPU",
		"GPU",
		"RAYTETRA",
		"RAYAABB",
		"RAYPRISM",
		"RAYCHULL",
		"RAYMESH",
		"RAYBILLB"
	};

	////////////
	// Header //
	////////////
	int tmp = renderModesDump_;
	GLuint64 result;
	if (!(append_ && exists)) {
		fileout << cpuName << "\n";
		fileout << gpuName << "\n";
		fileout << "runNum";
		for (int j = 0; j < RENDERMODE_NUM; j++) {
			if (tmp & 1) {
				fileout << "," << modes[j];
			}
			tmp = tmp >> 1;
		}
		fileout << "\n";
	}

	/////////////
	// Content //
	/////////////
	if (!average_) {
		// Iterate all queries and print the results sorted after the mode
		for (int i = 0; i < QUERY_COUNT; i++) {
			fileout << columnOne(i);
			tmp = renderModesDump_;
			for (int j = 0; j < RENDERMODE_NUM; j++) {
				if (tmp & 1) {
					glGetQueryObjectui64v(
						queries_[j][i], GL_QUERY_RESULT, &result);

					switch (testType_) {
						case TEST_TYPE::FTIME: 
							fileout << "," << result * 1.e-6; break;
						case TEST_TYPE::OCCL: fileout << "," << result; break;
					}
				}
				tmp = tmp >> 1;
			}
			fileout << "\n";
		}
	} else {
		tmp = renderModesDump_;
		fileout << columnOne(1);
		// Iterate all rendermodes and average the data
		for (int i = 0; i < RENDERMODE_NUM; i++) {
			double avg = 0.0;
			if (tmp & 1) {
				for (int j = 0; j < QUERY_COUNT; j++) {
					glGetQueryObjectui64v(
						queries_[i][j], GL_QUERY_RESULT, &result);
					switch (testType_) {
						case TEST_TYPE::FTIME: avg += result * 1.e-6; break;
						case TEST_TYPE::OCCL: avg += result; break;
					}
				}
				fileout << "," << (avg / QUERY_COUNT);
			}
			tmp = tmp >> 1;
		}
		fileout << std::endl;
	}
}

//=============================================================================
} // namespace betri
//=============================================================================