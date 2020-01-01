#include "AlgorithmTimer.hh"

#include <cassert>

void AlgorithmTimer::start(std::string msg)
{
	assert(!m_filename.empty());

	m_stream = std::ofstream(m_filename, std::ios_base::app);
	if (!msg.empty()) {
		m_stream << msg << '\n';
	}

	m_last = std::chrono::high_resolution_clock::now();
}

void AlgorithmTimer::lapStart(std::string msg)
{
	m_last = std::chrono::high_resolution_clock::now();
	if (!msg.empty()) {
		m_stream << " " << msg << '\n';
	}
}

void AlgorithmTimer::lapEnd(std::string msg)
{
	auto tmp = std::chrono::high_resolution_clock::now();
	auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(tmp- m_last);
	m_stream << diff.count() << " ms";
	if (!msg.empty()) {
		m_stream << " " << msg;
	}
	m_stream << '\n';
	m_last = tmp;
}

void AlgorithmTimer::end(std::string msg)
{
	lapEnd(msg);
	m_stream.flush();
	m_stream.close();
}
