#include "BezierOptions.hh"

namespace betri
{

static std::array<int, BezierOption::N> s_options;
static size_t s_degree = 3;

int option(BezierOption b)
{
	return s_options[b];
}

void option(BezierOption b, int value)
{
	s_options[b] = value;
}

void init()
{
	s_options.fill(0);
}

void globalDegree(size_t degree)
{
	s_degree = degree;
}

size_t globalDegree()
{
	return s_degree;
}

}