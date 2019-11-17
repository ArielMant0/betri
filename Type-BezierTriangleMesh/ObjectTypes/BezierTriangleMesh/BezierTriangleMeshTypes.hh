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

		void edgePoints(Point from, Point to, size_t deg, const std::vector<Point> &p)
		{
			assert(p.size() == deg + 1);

			int i0 = findClosestCorner(from, deg);
			int j0 = findClosestCorner(to, deg, i0);
			assert(i0 != j0);

			int min0, max0, mod;
			calcIndexMod(i0, j0, min0, max0, mod, deg);

			for (int i = i0, j = 0; j <= deg; ++j) {
				m_cps[i] = p[j];
				i += mod;
				mod = min0 == 0 && max0 == deg ? mod : mod-1;
			}
		}

		std::vector<Point> edgePoints(Point from, Point to, size_t deg) const
		{
			std::vector<Point> cps;
			cps.reserve(deg+1);

			int i0 = findClosestCorner(from, deg);
			int j0 = findClosestCorner(to, deg, i0);
			assert(i0 != j0);

			int min0, max0, mod;
			calcIndexMod(i0, j0, min0, max0, mod, deg);

			for (int i = i0, j = 0; j <= deg; ++j) {
				cps.push_back(m_cps[i]);
				i += mod;
				mod = min0 == 0 && max0 == deg ? mod : mod-1;
			}
			assert(cps.size() == deg + 1);

			return cps;
		}

		void prepare(const size_t size)
		{
			m_cps.resize(size);
		}

		void controlPoint(int index, Point point)
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

	private:

		int findClosestCorner(Point &p, size_t degree, int ignore=-1) const
		{
			constexpr double maxval = std::numeric_limits<double>::max();
			double d0 = ignore == 0 ? maxval : (p - m_cps.front()).norm();
			double d1 = ignore == degree ? maxval : (p - m_cps[degree]).norm();
			double d2 = ignore == m_cps.size()-1 ? maxval : (p - m_cps.back()).norm();

			if (d0 < d1 && d0 < d2)
				return 0;
			if (d1 < d0 && d1 < d2)
				return degree;

			return m_cps.size()-1;
		}

		void calcIndexMod(int i, int j, int &min, int &max, int &mod, size_t degree) const
		{
			min = i < j ? i : j;
			assert(min == 0 || min == degree);
			max = i < j ? j : i;
			assert(max == degree || max == m_cps.size() - 1);

			if (i == min) {
				mod = min == 0 ? (max == degree ? 1 : degree + 1) : degree;
			} else {
				mod = min == 0 && max != degree ? -2 : -1;
			}
		}
	};
};

//== TYPEDEFS =================================================================
