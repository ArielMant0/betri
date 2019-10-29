#include "NGonMapper.hh"

#include <cmath>

namespace betri
{

void NGonMapper::map(std::vector<Path*> &paths)
{
    // const Path path = connect(paths);

    const Scalar angle = (2.0 * M_PI) / paths.size();
    const Scalar sideLength = std::sin(M_PI / paths.size());
	const Point trans(0.5, 0.5, 0.0);

	const auto connectPaths = [&](Path *p0, Path *p1) {
		const bool reverse = id(p0->front()) == id(p1->front());
		reverse ? p0->insert(p0->begin(), p1->front()) : p0->push_back(p1->front());
	};

    // debug, make sure all paths overlap at the beginning/end
    for (size_t i = 0; i < paths.size(); ++i) {
		auto p0 = paths[(i > 0 ? i-1 : paths.size()-1)];
		auto p1 = paths[i];
		if (p1->front() != p0->back() && p1->front() != p0->front()) {
			connectPaths(p0, p1);
		}
    }

    for (size_t i = 0; i < paths.size(); ++i) {

        Path *path = paths[i];

        // check if we have to reverse iterate over the path
        const bool reverse = i > 0 && id(path->front()) != id(paths[i-1]->back());
        const long start = reverse ? path->size() - 1 : 0;
        const long end = reverse ? 0 : path->size() - 1;
        const long mod = reverse ? -1 : 1;

        // starting point is always on the unit circle
		Point initialT = Point(cos(angle*i), sin(angle*i), 0.) + trans, t = initialT;
        // norm factor depends on the side length of the ngon and the path length
        Scalar norm = sideLength / pathLength(path);
		Point first = m_mesh.point(path->at(start));

		size_t k = i < path->size() - 1 ? i + 1 : 0u;
		Point next = Point(cos(angle*k), sin(angle*k), 0.) + trans;
		Point dir = (t - next).normalize();

        for (long j = start; j != end; j+=mod) {
            VertexHandle vh = path->at(j);
            hmap(vh) = Vec2(t[0], t[1]);

            std::cerr << "\tcalculated uv = " << hmap(vh) << "\n";
            assert(std::isgreaterequal(hmap(vh)[0], 0.));
            assert(std::isgreaterequal(hmap(vh)[1], 0.));
            assert(std::islessequal(hmap(vh)[0] + hmap(vh)[1], 1.0));

			t = initialT + dir * (first - m_mesh.point(vh)).norm() * norm;
        }
    }
}

} // namespace betri
