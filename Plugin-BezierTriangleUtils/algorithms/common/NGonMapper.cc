#include "NGonMapper.hh"

#include <cmath>

namespace betri
{

void NGonMapper::map(std::vector<Path*> &paths)
{
    // const Path path = connect(paths);

    const Scalar angle = 360. / paths.size();
    const Scalar sideLength = 2.0 * std::sin(M_PI / paths.size());

    // debug, make sure all paths overlap at the beginning/end
    for (size_t i = 1; i < paths.size(); ++i) {
        assert(paths[i]->back() == paths[i-1]->at(0) || paths[i]->back() == paths[i-1]->back());
    }

    for (size_t i = 0; i < paths.size(); ++i) {

        Path *path = paths[i];

        // check if we have to reverse iterate over the path
        const bool reverse = i > 0 && id(path->at(0)) != id(paths[i-1]->back());
        const long start = reverse ? path->size() - 1 : 0;
        const long end = reverse ? 0 : path->size() - 1;
        const long mod = reverse ? -1 : 1;

        // starting point is always on the unit circle
		Scalar t = 0.0;
        Scalar tx = cos(angle * i);
        Scalar ty = sin(angle * i);
        // norm factor depends on the side length of the ngon and the path length
        Scalar norm = sideLength / pathLength(path);
        Point p;

        for (long j = start; j != end; j+=mod) {
            VertexHandle vh = path->at(j);
            hmap(vh) = Vec2(tx, ty);

            std::cerr << "\tcalculated uv = " << hmap(vh) << "\n";
            assert(std::isgreaterequal(hmap(vh)[0], 0.));
            assert(std::isgreaterequal(hmap(vh)[1], 0.));
            assert(std::islessequal(hmap(vh)[0] + hmap(vh)[1], 1.0));

            t += (p - m_mesh.point(vh)).norm() * norm;
            tx = 1. - t;
            ty = t;
            p = m_mesh.point(vh);
        }
    }
}

} // namespace betri
