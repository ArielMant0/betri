#include "NGonMapper.hh"

#include <cmath>

namespace betri
{

void betri::NGonMapper::mapTriangle(std::vector<Path*>& paths)
{
	const Point initial[3] = {
		Point(0.0, 0.0, 0.0),
		Point(0.5, 1.0, 0.0),
		Point(1.0, 0.0, 0.0)
	};

	bool lastReverse = false;
	for (size_t i = 0; i < paths.size(); ++i) {

		Path *path = paths[i];

		// check if we have to reverse iterate over the path
		const bool reverse = i > 0 && checkReverse(
			lastReverse,
			id(path->front()),
			id(path->back()),
			id(paths[i - 1]->front()),
			id(paths[i-1]->back())
		);
		const long start = reverse ? path->size() - 1 : 0;
		const long end = reverse ? -1 : path->size();
		const long mod = reverse ? -1 : 1;
		lastReverse = reverse;

		// starting point is always on the unit circle
		Point initialT = initial[i];
		Point t = Point(initialT);

		// norm factor depends on the side length of the ngon and the path length
		Point first = m_mesh.point(path->at(start));

		size_t k = i < paths.size() - 1 ? i + 1 : 0u;
		Scalar norm = (initial[i] - initial[k]).norm() / pathLength(path);
		Point dir = (initial[k] - t).normalize();

		for (long j = start; j != end; j += mod) {
			VertexHandle vh = path->at(j);
			hmap(vh) = Vec2(t[0], t[1]);

			std::cerr << "\tcalculated uv = " << hmap(vh) << "\n";
			assert(std::isgreaterequal(hmap(vh)[0], 0.));
			assert(std::isgreaterequal(hmap(vh)[1], 0.));
			assert(std::islessequal(hmap(vh)[0], 1.0));
			assert(std::islessequal(hmap(vh)[1], 1.0));

			t = initialT + dir * (first - m_mesh.point(vh)).norm() * norm;
		}
	}
}

void NGonMapper::map(std::vector<Path*> &paths)
{
	/*if (paths.size() == 3) {
		return mapTriangle(paths);
	}*/
    // const Path path = connect(paths);

	const Scalar ru = 0.5; // ri / std::cos(M_PI / paths.size()); // outer radius
    const Scalar angle = (2.0 * M_PI) / paths.size();
    const Scalar sideLength = 2.0 * ru * std::sin(M_PI / paths.size());
	const Point trans(ru, ru, 0.0);

	const auto connectPaths = [&](Path *p0, Path *p1) {
		const bool reverse = id(p0->front()) == id(p1->front());
		if (reverse) {
			p0->insert(p0->begin(), p1->front());
			assert(p0->front() == p1->front());
		} else {
			p0->push_back(p1->front());
			assert(p0->back() == p1->front());
		}
	};

    // debug, make sure all paths overlap at the beginning/end
    for (size_t i = 0; i < paths.size(); ++i) {
		auto p0 = paths[(i > 0 ? i-1 : paths.size()-1)];
		auto p1 = paths[i];
		if (p1->front() != p0->back() && p1->front() != p0->front()) {
			// TODO: not good bc paths are connected to several other paths?
			connectPaths(p0, p1);
		}
    }

	bool lastReverse = false;
	for (size_t i = 0; i < paths.size(); ++i) {

        Path *path = paths[i];

        // check if we have to reverse iterate over the path
		const bool reverse = i > 0 && checkReverse(
			lastReverse,
			id(path->front()),
			id(path->back()),
			id(paths[i - 1]->front()),
			id(paths[i - 1]->back())
		);
		const long start = reverse ? path->size() - 1 : 0;
		const long end = reverse ? -1 : path->size();
		const long mod = reverse ? -1 : 1;
		lastReverse = reverse;

        // starting point is always on the unit circle
		Point initialT = trans + Point(ru * sin(angle*i), ru * cos(angle*i), 0.);
		Point t = Point(initialT);

        // norm factor depends on the side length of the ngon and the path length
        Scalar norm = sideLength / pathLength(path);
		Point first = m_mesh.point(path->at(start));

		size_t k = i < paths.size()-1 ? i + 1 : 0u;
		Point next = trans + Point(ru * sin(angle*k), ru * cos(angle*k), 0.);
		Point dir = (next-t).normalize();

        for (long j = start; j != end; j+=mod) {
            VertexHandle vh = path->at(j);
            hmap(vh) = Vec2(t[0], t[1]);

            std::cerr << "\tcalculated uv = " << hmap(vh) << "\n";
            assert(std::isgreaterequal(hmap(vh)[0], 0.));
            assert(std::isgreaterequal(hmap(vh)[1], 0.));
            assert(std::islessequal(hmap(vh)[0], 1.0));
            assert(std::islessequal(hmap(vh)[1], 1.0));

			t = initialT + dir * (first - m_mesh.point(vh)).norm() * norm;
        }
    }
}

} // namespace betri
