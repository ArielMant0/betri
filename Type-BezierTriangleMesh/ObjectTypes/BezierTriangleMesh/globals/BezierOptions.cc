#include "BezierOptions.hh"

namespace betri
{

std::array<int, BezierOption::N> s_options;

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

//int BezierOptionMng::option(BezierOption b)
//{
//	return s_options[b];
//}
//
//void BezierOptionMng::option(BezierOption b, int value)
//{
//	s_options[b] = value;
//}
//
//void BezierOptionMng::init()
//{
//	s_options.fill(0);
}

}