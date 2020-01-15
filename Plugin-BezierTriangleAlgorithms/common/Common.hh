#pragma once

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

namespace betri
{

// Useful typedefs for all algorithms
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

// for voronoi meshing
using ID = int;

struct TriToVertex
{
	std::vector<VertexHandle> inner; // inner vertices
	std::array<ID, 3> ids{ { -1, -1, -1 } };// boundary numbers (shortest paths)
	size_t boundarySize = 0;

	const ID& operator[](size_t index) const
	{
		assert(index <= 2);
		return ids[index];
	}

	ID& operator[](size_t index)
	{
		assert(index <= 2);
		return ids[index];
	}

	void addBoundarySize(size_t size)
	{
		boundarySize += size;
	}

	bool isRegion(ID r0, ID r1, ID r2) const
	{
		return (ids[0] == r0 || ids[1] == r0 || ids[2] == r0) &&
			(ids[0] == r1 || ids[1] == r1 || ids[2] == r1) &&
			(ids[0] == r2 || ids[1] == r2 || ids[2] == r2);
	}

};

struct VertexToTri
{
	FaceHandle face; // delaunay triangle in control mesh (invalid if border)
	ID id1 = -1, id2 = -1;
	Vec2 uv; // parameterization

	void setUV(double u, double v)
	{
		uv[0] = u;
		uv[1] = v;
	}

	void setFace(const FaceHandle f)
	{
		face = f;
		id1 = f.idx();
		id2 = -1;
	}

	void setBorder(ID i1, ID i2)
	{
		id1 = i1;
		id2 = i2;
	}

	bool isBorder() const
	{
		return id1 >= 0 && id2 >= 0;
	}

	bool isBorderOf(const TriToVertex &ttv) const
	{
		bool id1Found = false, id2Found = false;

		for (size_t i = 0; i < 3; ++i) {
			if (ttv[i] == id1) id1Found = true;
			else if (ttv[i] == id2) id2Found = true;
		}

		return id1Found && id2Found;
	}

	bool isInnerOf(const TriToVertex &ttv) const
	{
		for (size_t i = 0; i < 3; ++i) {
			if (ttv[i] == id1) return true;
		}

		return false;
	}

	const Vec2::value_type& operator[](size_t index) const
	{
		assert(index < 2);
		return uv[index];
	}

	Vec2::value_type& operator[](size_t index)
	{
		assert(index < 2);
		return uv[index];
	}
};

struct FitCollection
{
	std::vector<Point> points;
	FaceHandle face;

	void add(Point &p)
	{
		points.push_back(p);
	}

	size_t size() const
	{
		return points.size();
	}

	Point& operator[](size_t index)
	{
		return points[index];
	}
};

inline size_t pointsFromDegree(const size_t degree)
{
	return (degree + 1)*(degree + 2) / 2;
}

inline Scalar eval(int i, int j, Vec2 bary, unsigned int degree)
{
	int k = (int)degree - i - j;
	double w = 1.0 - bary[0] - bary[1];

	return FACTORIALS[degree] / (FACTORIALS[i] * FACTORIALS[j] * FACTORIALS[k])
		* std::pow(bary[0], i) * std::pow(bary[1], j) * std::pow(w, k);
}

inline Point evalSurface(const std::vector<Point> &cps, const Vec2 &bary, const unsigned int degree)
{
	Point point(0.0);

	for (int i = 0; i <= degree; i++) {
		for (int j = 0; j + i <= degree; j++) {
			point += cps[cpIndex(i, j, degree)] * eval(i, j, bary, degree);
		}
	}
	return point;
}

}