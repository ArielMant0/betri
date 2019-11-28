#pragma once

//== INCLUDES =================================================================

#include <Type-OpenMesh/ObjectTypes/TriangleMesh/TriangleMeshTypes.hh>

//#include <iostream>

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

		void align(size_t degree, Point &p0, Point &p1, Point p2)
		{
			int c0, c1, c2;
			std::tie(c0, c1, c2) = findClosestCorners(degree, p0, p1, p2);

			assert(c0 != c1 && c1 != c2 && c0 != c2);

			// control points are already aligned
			if (c0 == 0 && c1 == degree && c2 == m_cps.size()-1) {
				return;
			}

			std::vector<Point> aligned(m_cps.size());

			int min0, max0, mod;
			// edge 1
			calcIndexMod(c0, c1, min0, max0, mod, degree);

			for (int i = c0, j = 0; j <= degree; ++j) {
				aligned[j] = m_cps[i];
				i += mod;
				mod = min0 == 0 && max0 == degree ? mod : mod - 1;
			}

			// edge 2
			calcIndexMod(c1, c2, min0, max0, mod, degree);

			int iterMod = degree;
			for (int i = c1, j = degree; j < m_cps.size(); ) {
				aligned[j] = m_cps[i];
				i += mod;
				mod = min0 == 0 && max0 == degree ? mod : mod - 1;
				j += iterMod;
				iterMod = std::max(iterMod - 1, 1);
			}

			// edge 3
			calcIndexMod(c2, c0, min0, max0, mod, degree);

			iterMod = -2;
			for (int i = c2, j = m_cps.size() - 1; j >= 0; ) {
				aligned[j] = m_cps[i];
				i += mod;
				mod = min0 == 0 && max0 == degree ? mod : mod - 1;
				j += iterMod;
				iterMod--;
			}

			m_cps = std::move(aligned);
		}

		void edgePoints(const int from, const int to, const size_t deg, const std::vector<Point> &p)
		{
			assert(p.size() == deg + 1);

			int end = m_cps.size() - 1;

			int i0 = from == 0 ? 0 : (from == 1 ? deg : end);
			int j0 = to == 0 ? 0 : (to == 1 ? deg : end);

			int min0, max0, mod;
			calcIndexMod(i0, j0, min0, max0, mod, deg);

			for (int i = i0, j = 0; j <= deg; ++j) {
				m_cps[i] = p[j];
				i += mod;
				mod = min0 == 0 && max0 == deg ? mod : mod-1;
			}
		}

		std::vector<Point> edgePoints(const int from, const int to, const size_t deg) const
		{
			std::vector<Point> cps;
			cps.reserve(deg+1);

			int end = m_cps.size() - 1;

			int i0 = from == 0 ? 0 : (from == 1 ? deg : end);
			int j0 = to == 0 ? 0 : (to == 1 ? deg : end);

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

		std::tuple<int,int> findClosestEdgeCorners(
			const size_t degree,
			const Point &p0,
			const Point &p1
		) const {

			constexpr double maxval = std::numeric_limits<double>::max();
			int end = m_cps.size() - 1;

			std::array<double, 3> d0 = { {
				(p0 - m_cps[0]).norm(),
				(p0 - m_cps[degree]).norm(),
				(p0 - m_cps[end]).norm()
			} };
			std::array<double, 3> d1 = { {
				(p1 - m_cps[0]).norm(),
				(p1 - m_cps[degree]).norm(),
				(p1 - m_cps[end]).norm()
			} };

			int i = std::min_element(d0.begin(), d0.end()) - d0.begin();
			int j = std::min_element(d1.begin(), d1.end()) - d1.begin();

			if (i == j) {
				findNextMin(i, j, d0, d1);
			}
			assert(i != j);

			i = i == 0 ? 0 : (i == 1 ? degree : end);
			j = j == 0 ? 0 : (j == 1 ? degree : end);

			return { i, j };
		}

		std::tuple<int, int, int> findClosestCorners(
			const size_t degree,
			const Point &p0, const Point &p1, const Point &p2
		) {

			int end = m_cps.size() - 1;

			std::array<double, 3> d0 = { {
				(p0 - m_cps[0]).norm(),
				(p0 - m_cps[degree]).norm(),
				(p0 - m_cps[end]).norm()
			} };
			std::array<double, 3> d1 = { {
				(p1 - m_cps[0]).norm(),
				(p1 - m_cps[degree]).norm(),
				(p1 - m_cps[end]).norm()
			} };
			std::array<double, 3> d2 = { {
				(p2 - m_cps[0]).norm(),
				(p2 - m_cps[degree]).norm(),
				(p2 - m_cps[end]).norm()
			} };

			int i = std::min_element(d0.begin(), d0.end()) - d0.begin();
			int j = std::min_element(d1.begin(), d1.end()) - d1.begin();
			int k = std::min_element(d2.begin(), d2.end()) - d2.begin();

			while (i == j || i == k || j == k) {
				if (i == j) {
					findNextMin(i, j, d0, d1);
				}
				if (i == k) {
					findNextMin(i, k, d0, d2);
				}
				if (j == k) {
					findNextMin(j, k, d1, d2);
				}
			}

			assert(i != j && i != k && j != k);

			i = i == 0 ? 0 : (i == 1 ? degree : end);
			j = j == 0 ? 0 : (j == 1 ? degree : end);
			k = k == 0 ? 0 : (k == 1 ? degree : end);

			return { i, j, k };
		}

	private:

		static void findNextMin(int &i, int &j, std::array<double, 3> &d0, std::array<double, 3> &d1)
		{
			if (i != j) return;

			constexpr double maxval = std::numeric_limits<double>::max();

			if (d0[i] < d1[j]) {
				d0[i] = maxval;
				i = std::min_element(d0.begin(), d0.end()) - d0.begin();
			} else {
				d1[j] = maxval;
				j = std::min_element(d1.begin(), d1.end()) - d1.begin();
			}
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
