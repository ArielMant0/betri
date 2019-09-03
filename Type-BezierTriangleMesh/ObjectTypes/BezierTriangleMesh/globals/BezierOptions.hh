#pragma once

#include <array>

namespace betri
{

enum BezierOption : int
{
	TESSELLATION_TYPE = 0,
	TESSELLATION_AMOUNT,
	N
};


extern std::array<int, BezierOption::N> s_options;

int option(BezierOption b);

void option(BezierOption b, int value);

void init();

//class BezierOptionMng
//{
//public:
//
//	static int option(BezierOption b);
//
//	static void option(BezierOption b, int value);
//
//	static void init();
//
//private:
//
//	static std::array<int, BezierOption::N> s_options;
//};

}

