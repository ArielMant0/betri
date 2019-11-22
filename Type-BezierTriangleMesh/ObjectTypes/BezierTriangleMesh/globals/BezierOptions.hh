#pragma once

#include <array>

namespace betri
{

// TODO tesselation one or two l?
enum BezierOption : int
{
	TESSELLATION_TYPE = 0, // TODO this should propably not be called tesselation type
	TESSELLATION_AMOUNT,
	TESSELLATION_ADAPTIVE,
	BOUNDING_VOLUME,
	SHOW_BOUNDING_VOLUME,
	B_ERROR,
	D_ERROR,
	NEWTON_IT_COUNT,
	VISUALISATION_MODE,
	N
};

// TODO oke so?
enum TESSELLATION_TYPE : int
{
	NONE = 0,
	CPU,
	GPU,
	RAYTRACING
};

enum VIS_MODE : int
{
	PHONGCOLOR = 0,
	COLOR,
	NORMAL,
	DEPTH,
	CURVATURE
};

//extern std::array<int, BezierOption::N> s_options;

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

