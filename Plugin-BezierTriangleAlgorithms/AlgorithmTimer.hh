#pragma once

#include <string>
#include <fstream>
#include <chrono>

class AlgorithmTimer
{
public:

	AlgorithmTimer() : m_filename("algorithm-times.txt") {}

	void filename(std::string name)
	{
		m_filename = std::move(name);
	}

	std::string filename() const
	{
		return m_filename;
	}

	void start(std::string msg = {});

	void lapStart(std::string msg = {});
	void lapEnd(std::string msg = {});

	void end(std::string msg = {});

private:

	std::string m_filename;
	std::ofstream m_stream;

	std::chrono::time_point<std::chrono::high_resolution_clock> m_last;
};