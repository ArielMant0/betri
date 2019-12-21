#pragma once

#include "BoundaryMapper.hh"

#include <OpenMesh/Core/Utils/Property.hh>

namespace betri
{

template <typename T>
class NGonMapper : public BoundaryMapper<T>
{
public:

	explicit NGonMapper(BezierTMesh &mesh, OpenMesh::VPropHandleT<T> &uvProp) :
		BoundaryMapper(mesh, uvProp) {}

	void map(std::vector<Path*> &paths) override
	{
		// make sure all paths overlap at the beginning/end
		for (size_t i = 0; i < paths.size(); ++i) {
			auto p0 = paths[(i > 0 ? i - 1 : paths.size() - 1)];
			auto p1 = paths[i];
			assert(p0->back() == p1->front());
		}

		if (paths.size() == 3) {
			return mapTriangle(paths);
		}

		const Scalar ru = 0.5; // ri / std::cos(M_PI / paths.size()); // outer radius
		const Scalar angle = (2.0 * M_PI) / paths.size();
		const Scalar sideLen = 2.0 * ru * std::sin(M_PI / paths.size());
		const Vec2 trans(ru, ru);

		for (size_t i = 0; i < paths.size(); ++i) {

			Path *path = paths[i];

			// starting point is always on the unit circle
			Vec2 initialT = trans + Vec2(ru * sin(angle*i), ru * cos(angle*i));
			Vec2 t = initialT;

			// norm factor depends on the side length of the ngon and the path length
			Scalar norm = sideLen / pathLength(path);
			Point first = m_mesh.point(path->front());

			size_t k = i < paths.size() - 1 ? i + 1 : 0u;
			Vec2 next = trans + Vec2(ru * sin(angle*k), ru * cos(angle*k));
			Vec2 dir = (next - t).normalize();

			for (size_t j = 0; j < path->size(); ++j) {
				VertexHandle vh = path->at(j);

				t = j == 0 ? t : initialT + dir * (first - m_mesh.point(vh)).norm() * norm;

				hmap(vh)[0] = t[0];
				hmap(vh)[1] = t[1];

				assert(std::isgreaterequal(hmap(vh)[0], 0.));
				assert(std::isgreaterequal(hmap(vh)[1], 0.));
				assert(std::islessequal(hmap(vh)[0], 1.0));
				assert(std::islessequal(hmap(vh)[1], 1.0));
			}
		}
	}

	Vec2 middle() override { return Vec2(0.33f, 0.33f); }

	static Vec2 middle(size_t n)
	{
		switch (n) {
			case 3: return middle();
			default: return Vec2(0.5f, 0.5f);
		}
	}

	static Scalar perimeter(size_t n)
	{
		return n * sideLength(n);
	}

	static Scalar sideLength(size_t n)
	{
		return std::sin(angle(n) / n);
	}

	static Scalar angle(size_t n)
	{
		return (2.0 * M_PI) / n;
	}

	static Vec2 corner(size_t index, size_t n)
	{
		const Scalar angle = (2.0 * M_PI) / n;

		return cornerFromAngle(index*angle);
	}

	static Vec2 cornerFromAngle(Scalar angle)
	{
		const Scalar ru = 0.5; // outer radius
		const Vec2 trans(ru, ru);

		return trans + Vec2(ru * std::cos(angle), ru * std::sin(angle));
	}

private:

	void mapTriangle(std::vector<Path*> &paths)
	{
		const Vec2 initial[3] = {
			Vec2(1.0, 0.0),
			Vec2(0.0, 1.0),
			Vec2(0.0, 0.0)
		};

		for (size_t i = 0; i < paths.size(); ++i) {

			Path *path = paths[i];

			// starting point is always on the unit circle
			Vec2 initialT = initial[i];
			Vec2 t = initialT;

			// norm factor depends on the side length of the ngon and the path length
			Point p = m_mesh.point(path->front());

			size_t k = i < paths.size() - 1 ? i + 1 : 0u;
			Scalar len = pathLength(path), curLen = 0.;
			Scalar norm = (initial[i] - initial[k]).norm() / len;
			Vec2 dir = (initial[k] - t).normalize();

			for (size_t j = 0; j < path->size(); ++j) {
				VertexHandle vh = path->at(j);

				curLen += (p - m_mesh.point(vh)).norm();
				t = j == 0 ? t : initialT + dir * curLen * norm;
				clamp(t);

				p = m_mesh.point(vh);

				hmap(vh)[0] = t[0];
				hmap(vh)[1] = t[1];

				assert(std::isgreaterequal(hmap(vh)[0], 0.));
				assert(std::isgreaterequal(hmap(vh)[1], 0.));
				assert(std::islessequal(hmap(vh)[0], 1.0));
				assert(std::islessequal(hmap(vh)[1], 1.0));
			}
		}
	}

};

}
