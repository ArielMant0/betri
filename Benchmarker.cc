///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////
#include "Benchmarker.hh"
#include <cctype>

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
		// start timer query
		if (occlQuery_)
			glBeginQuery(GL_SAMPLES_PASSED, queries_[shifted_ - 1][current_query_]);
		else
			glBeginQuery(GL_TIME_ELAPSED, queries_[shifted_-1][current_query_]);
	}
}

void Benchmarker::endFrame()
{
	if (active()) {
		// end timer query
		if (occlQuery_)
			glEndQuery(GL_SAMPLES_PASSED);
		else
			glEndQuery(GL_TIME_ELAPSED);

		/*
		// display timer query results from querycount frames before
		if (GL_TRUE == glIsQuery(queries_[current_query_])) {
			GLuint64 result;
			glGetQueryObjectui64v(queries_[current_query_], GL_QUERY_RESULT, &result);
			std::cerr << result * 1.e-6 << " ms/frame" << std::endl;
		}
		*/
		// advance query counter
		if (current_query_ + 1 == QUERY_COUNT) {
			current_query_ = 0;
			advanceRenderMode();
		} else {
			current_query_++;
		}
	}
}

void Benchmarker::active(bool activate)
{
	// TODO current_query_ = 0
	active_ = activate;
}

bool Benchmarker::active() const
{
	// TODO return current_query_ < QUERY_COUNT
	return active_;
}

bool Benchmarker::update()
{
	bool tmp = updateBuffers_;
	updateBuffers_ = false;
	return tmp;
}

void Benchmarker::occlQuery(bool state)
{
	occlQuery_ = state;
}

void Benchmarker::average(bool state)
{
	average_ = state;
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

void Benchmarker::advanceRenderMode()
{
	updateBuffers_ = true;

	if (renderModes_ == 0) {
		active(false);
		dumpToFile();
		activeRMode_ = 0;
		activeBVol_ = 0;
		shifted_ = 0;
		return;
	}

	for (; !(renderModes_ & 1); shifted_++) {
		renderModes_ = renderModes_ >> 1;
	}

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

void Benchmarker::dumpToFile()
{
	// https://www.khronos.org/opengl/wiki/OpenGL_Context#Context_information_queries
	// https://stackoverflow.com/questions/42245870/how-to-get-the-graphics-card-model-name-in-opengl-or-win32
	// https://stackoverflow.com/questions/83439/remove-spaces-from-stdstring-in-c
	//const GLubyte* vendor = glGetString(GL_VENDOR);
	const GLubyte* renderer = glGetString(GL_RENDERER);
	std::string gpuName(reinterpret_cast<const char*>(renderer));
	gpuName.erase(std::remove_if(gpuName.begin(), gpuName.end(), std::isspace), gpuName.end());
	gpuName = gpuName.substr(0, gpuName.find("/", 0));

	average_ = false;

	std::string fileDir = "C:/Users/DoktorManto/Uni/Masterarbeit/masterthesis/rene/data/";
	std::ofstream fileout(
		fileDir + gpuName + 
		"-" + std::to_string(average_) +
		"-" + std::to_string(occlQuery_) +
		".dat"
	);

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
	fileout << "runNum";
	for (int j = 0; j < RENDERMODE_NUM; j++) {
		if (tmp & 1) {
			fileout << "," << modes[j];
		}
		tmp = tmp >> 1;
	}
	fileout << "\n";

	/////////////
	// Content //
	/////////////
	if (!average_) {
		for (int j = 0; j < QUERY_COUNT; j++) {
			fileout << std::to_string(j + 1);
			tmp = renderModesDump_;
			for (int i = 0; i < RENDERMODE_NUM; i++) {
				if (tmp & 1) {
					if (occlQuery_) {
						glGetQueryObjectui64v(queries_[i][j], GL_QUERY_RESULT, &result);
						fileout << "," << result;
					} else {
						glGetQueryObjectui64v(queries_[i][j], GL_QUERY_RESULT, &result);
						fileout << "," << result * 1.e-6;
					}
				}
				tmp = tmp >> 1;
			}
			fileout << "\n";
		}
	} else {
		tmp = renderModesDump_;
		fileout << "1";
		for (int i = 0; i < RENDERMODE_NUM; i++) {
			float avg = 0.0;
			if (tmp & 1) {
				for (int j = 0; j < QUERY_COUNT; j++) {
					if (occlQuery_) {
						glGetQueryObjectui64v(queries_[i][j], GL_QUERY_RESULT, &result);
						fileout << "," << result;
					} else {
						glGetQueryObjectui64v(queries_[i][j], GL_QUERY_RESULT, &result);
						fileout << "," << result * 1.e-6;
					}
				}
				fileout << "," << (avg / QUERY_COUNT);
			}
			tmp = tmp >> 1;
		}
	}
}

//=============================================================================
} // namespace betri
//=============================================================================