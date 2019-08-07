#pragma once

//== INCLUDES =================================================================

#include <Type-OpenMesh/ObjectTypes/TriangleMesh/TriangleMeshTypes.hh>

struct BezierTriangleTraits : public TriTraits
{

	FaceTraits {
	private:
		// all control points of the face (ccw starting from he)
		// IDEA: is it better to store 2 float coeffs (linear combination)?
		std::vector<Point> m_cps;

	public:

		void setControlPoints(std::vector<Point> &points)
		{
			m_cps = std::move(points);
		}

		void setPoint(int index, Point &point)
		{
			if (m_cps.size() <= index) {
				m_cps.push_back(point);
			} else {
				m_cps[index] = point;
			}
		}

		void addPoint(Point &point)
		{
			m_cps.push_back(point);
		}

		// returns an iterator to the beginning of all control points
		auto cpBegin() const
		{
			return m_cps.begin();
		}

		// returns an iterator to the end of all control points
		auto cpEnd() const
		{
			return m_cps.end();
		}

		Point getCPoint(int index) const
		{
			return m_cps[index];
		}

		unsigned int degree() const
		{
			return m_cps.size() / 3;
		}
	};
};

//== TYPEDEFS =================================================================

//using BezierTMesh = OpenMesh::TriMesh_ArrayKernelT<BezierTriangleTraits>;

class BezierTMesh : public OpenMesh::TriMesh_ArrayKernelT<BezierTriangleTraits>
{
public:

	void degreeElevation(BezierTMesh::FaceHandle &fh);

	void degreeReduction(BezierTMesh::FaceHandle &fh);
};