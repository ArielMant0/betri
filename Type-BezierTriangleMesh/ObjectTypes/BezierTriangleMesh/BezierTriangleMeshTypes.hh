#pragma once

//== INCLUDES =================================================================

#include <Type-OpenMesh/ObjectTypes/TriangleMesh/TriangleMeshTypes.hh>

//#include <OpenFlipper/libs_required/OpenMesh/src/OpenMesh/Core/Mesh/TriConnectivity.hh>

struct BezierTriangleTraits : public TriTraits
{
	FaceTraits {
	private:
		// all control points of the face (ccw starting from he)
		// IDEA: is it better to store 2 float coeffs (linear combination)?
		std::vector<Point> m_cps;

	public:

		void points(const std::vector<Point> &points)
		{
			m_cps = points;
		}

		std::vector<Point>& points()
		{
			return m_cps;
		}

		void controlPoint(int index, Point &point)
		{
			if (m_cps.size() <= index) {
				m_cps.push_back(point);
			} else {
				m_cps[index] = point;
			}
		}

		Point controlPoint(int index) const
		{
			return m_cps[index];
		}

		void addPoint(Point &point)
		{
			m_cps.push_back(point);
		}

		void clear()
		{
			m_cps.clear();
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
	};
};

//== TYPEDEFS =================================================================
