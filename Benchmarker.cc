///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////
#include "Benchmarker.hh"


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
		glBeginQuery(GL_TIME_ELAPSED, queries_[shifted_-1][current_query_]);
	}
}

void Benchmarker::endFrame()
{
	if (active()) {
		// end timer query
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
		default: break;
	}
	shifted_++;
	renderModes_ = renderModes_ >> 1;
	assert(shifted_ < 1000);
}

void Benchmarker::dumpToFile()
{
	std::string fileDir = "C:/Users/DoktorManto/Desktop/";
	fileName_ = "";
	std::ofstream fileout(fileDir + fileName_ + "_output.txt");

	if (!fileout) {
		std::cerr << "Error open fileout!" << std::endl;
		return;
	}

	GLuint64 result;
	fileout << "runNum" << "," << "CPU" << "\n";
	for (int j = 0; j < QUERY_COUNT; j++) {
		fileout << std::to_string(j + 1);
		int tmp = renderModesDump_;
		for (int i = 0; i < RENDERMODE_NUM; i++) {
			if (tmp & 1) {
				glGetQueryObjectui64v(queries_[i][j], GL_QUERY_RESULT, &result);
				fileout << "," << result * 1.e-6;
			}
			tmp = tmp >> 1;
		}
		fileout << "\n";
	}
}

//=============================================================================
} // namespace betri
//=============================================================================