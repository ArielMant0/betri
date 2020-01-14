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
	if (!msg.empty()) {
		m_stream << " " << msg << '\n';
	}
	m_last = std::chrono::high_resolution_clock::now();
}

void AlgorithmTimer::lapEnd(std::string msg)
{
	auto delta = std::chrono::high_resolution_clock::now() - m_last;
	// write time in several formats (milliseconds, seconds, minutes, hours)
	auto diffms = std::chrono::duration_cast<std::chrono::milliseconds>(delta);
	auto diffs = std::chrono::duration_cast<std::chrono::seconds>(delta);
	auto diffm = std::chrono::duration_cast<std::chrono::minutes>(delta);
	auto diffh = std::chrono::duration_cast<std::chrono::hours>(delta);
	m_stream << diffms.count() << " ms " << diffs.count() << " sec ";
	m_stream << diffm.count() << " min " << diffh.count() << " h";
	// write msg
	if (!msg.empty()) {
		m_stream << " " << msg;
	}
	m_stream << '\n';
	m_last = std::chrono::high_resolution_clock::now();
}

void AlgorithmTimer::end(std::string msg)
{
	lapEnd(msg);
	m_stream << '\n';
	m_stream.flush();
	m_stream.close();
}
