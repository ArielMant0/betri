#pragma once

#include <array>

#include <OpenFlipper/common/ObjectTypeDLLDefines.hh>

namespace betri
{

enum BezierOption : int
{
	TESSELLATION_TYPE = 0,
	TESSELLATION_AMOUNT,
	TESSELLATION_ADAPTIVE,
	BOUNDING_VOLUME,
	SHOW_BOUNDING_VOLUME,
	CULL_FACES,
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
	UV,
	CURVATURE
};

int option(BezierOption b);

void option(BezierOption b, int value);

void init();

OBJECTTYPEDLLEXPORTONLY void globalDegree(size_t degree);

OBJECTTYPEDLLEXPORTONLY size_t globalDegree();

}

