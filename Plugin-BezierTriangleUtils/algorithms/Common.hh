#pragma once

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

namespace betri
{

// Useful typedefs
using VertexHandle = BezierTMesh::VertexHandle;
using EdgeHandle = BezierTMesh::EdgeHandle;
using FaceHandle = BezierTMesh::FaceHandle;
using HalfedgeHandle = BezierTMesh::HalfedgeHandle;
using Scalar = BezierTMesh::Scalar;
using Point = BezierTMesh::Point;
using Color = BezierTMesh::Color;

using VertexIter = BezierTMesh::VertexIter;
using VertexVertexIter = BezierTMesh::VertexVertexIter;
using VertexFaceIter = BezierTMesh::VertexFaceIter;
using FaceVertexIter = BezierTMesh::FaceVertexIter;
using EdgeIter = BezierTMesh::EdgeIter;
using Vec2 = ACG::VectorT<Scalar, 2>;
using Vec3 = ACG::VectorT<Scalar, 3>;

struct TriToVertex
{
	std::vector<VertexHandle> inner; // inner vertices
	size_t ids[3] = { 0, 0, 0 };// boundary numbers (shortest paths)

	const size_t& operator[](size_t index) const
	{
		assert(index <= 2);
		return ids[index];
	}

	size_t& operator[](size_t index)
	{
		assert(index <= 2);
		return ids[index];
	}
};

struct VertexToTri
{
	FaceHandle face; // delaunay triangle in control mesh (invalid if border)
	Vec2 uv; // parameterization

	void set(double u, double v)
	{
		uv[0] = u;
		uv[1] = v;
	}

	const double& operator[](size_t index) const
	{
		assert(index < 2);
		return uv[index];
	}

	double& operator[](size_t index)
	{
		assert(index < 2);
		return uv[index];
	}
};

static constexpr float FACTORIALS[13] = {
	1, 1, 2, 6, 24, 120, // 0, 1, 2, 3, 4, 5
	720, 5040, 40320, 362880, 3628800, // 6, 7, 8, 9, 10
	39916800, 479001600
};

inline Scalar eval(int i, int j, float u, float v, unsigned int degree)
{
	int k = degree - i - j;
	float w = 1.f - u - v;

	return FACTORIALS[degree] / (FACTORIALS[i] * FACTORIALS[j] * FACTORIALS[k])
		* std::pow(u, i) * std::pow(v, j) * std::pow(w, k);
}

}