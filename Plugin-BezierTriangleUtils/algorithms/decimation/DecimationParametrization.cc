#include "DecimationParametrization.hh"

namespace betri
{

void DecimationParametrization::prepare()
{
	// note: we take the degree+1 so we always sample inside the triangle
	m_sampleUVs = getSampleUVs(m_mesh.degree()+2);

	std::cerr << "calculated sample uvs:\n";
	for (Vec2 &uv : m_sampleUVs) {
		std::cerr << '\t' << uv << '\n';
	}
}

void DecimationParametrization::cleanup()
{}

bool DecimationParametrization::solve()
{
	return false;
}

bool DecimationParametrization::solveLocal(
	const VertexHandle from,
	const VertexHandle to,
	std::vector<FitCollection>& fitColl,
	const bool print
) {
	HalfedgeHandle collapse = m_mesh.find_halfedge(from, to);
	assert(collapse.is_valid());
	FaceHandle f0 = m_mesh.face_handle(collapse), f1 = m_mesh.opposite_face_handle(collapse);

	size_t n = m_mesh.valence(from);

	//std::cerr << "parametrization with valence " << n << std::endl;

	std::unordered_map<FaceHandle, NGonFace> faces;
	std::unordered_map<FaceHandle, NGonFace> facesOrig;

	size_t idx = 0;

	// calculate uv coordinate of vertex to collapse into
	Vec2 toCorner;
	for (auto h_it = m_mesh.cvoh_begin(from); h_it != m_mesh.cvoh_end(from); ++h_it, ++idx) {
		if (m_mesh.to_vertex_handle(*h_it) == to) {
			toCorner = m_mapper.corner(idx, n);
			break;
		}
	}

	//std::cerr << "settings ngon faces:\n";

	Vec2 middle = m_mapper.middle(n);

	idx = 0;
	// calculate uv corners for each triangle *after* collapse
	for (auto h_it = m_mesh.cvoh_begin(from); h_it != m_mesh.cvoh_end(from); ++h_it, ++idx) {

		auto next = std::next(h_it);
		if (next == m_mesh.cvoh_end(from)) {
			next = m_mesh.cvoh_begin(from);
		}

		VertexHandle v0 = m_mesh.to_vertex_handle(*h_it);
		VertexHandle v1 = m_mesh.to_vertex_handle(*next);

		FaceHandle f = m_mesh.face_handle(*h_it);
		if (!m_mesh.adjToVertex(f, v1)) {
			f = m_mesh.opposite_face_handle(*h_it);
			assert(m_mesh.adjToVertex(f, v1));
		}

		if (v0 != to && v1 != to) {
			assert(f != f0 && f != f1);
			// the face after the halfedge collapse
			NGonFace corners{{
				m_mapper.corner(idx, n),
				m_mapper.corner((idx+1) % n, n),
				toCorner
			}};
			faces.insert({ f, corners });
			facesOrig.insert({ f, {
				corners[0],
				corners[1],
				middle
			} });

		} else {
			facesOrig.insert({ f, {
				m_mapper.corner(idx, n),
				m_mapper.corner((idx + 1) % n, n),
				middle
			} });
		}
	}
	assert(faces.size() == (n - 2));

	/*if (print) {
		for (auto &stuff : facesOrig) {
			std::cerr << "\tface " << stuff.first << "\n";
			for (auto &pair : stuff.second) {
				std::cerr << "\t\t" << pair << "\n";
			}
		}
	}*/

	unsigned int degree = m_mesh.degree();

	// for each face that remains:
	// - for a give number of sample uv points in a triangle
	//  - find target face
	//  - evaluate the face's surface at the given uv position and store that combination
	for (auto &pair : faces) {

		// only calculate for remaining faces

		FitCollection fit;
		fit.face = pair.first;

		Point faceBary;

		for (Vec2 &uv : m_sampleUVs) {
			Vec2 faceUV = trianglePoint(uv, pair.second);
			// find the non-collapsed face in which this uv point lies
			FaceHandle target = findTargetFace(faceUV, faceBary, facesOrig);
			assert(target.is_valid());
			// sample the target face at its uv position
			fit.add(
				evalSurface(m_mesh.data(target).points(), faceBary, degree),
				uv
			);
			/*if (print) {
				std::cerr << "adding surface sample for target "<<  target << '\n';
				std::cerr << "\tuv " << fit.uvs.back().first << '\n';
				std::cerr << "\tposition " << fit.uvs.back().second << '\n';
				std::cerr << "\tface uv " << faceUV << '\n';
			}*/
		}

		fitColl.push_back(fit);
	}

	return true;
}

std::vector<Vec2> DecimationParametrization::getSampleUVs(size_t degree)
{
	std::vector<Vec2> uvs;
	uvs.reserve(pointsFromDegree(degree));

	Scalar inc = 1.0 / degree;

	for (size_t i = 0; i <= degree; ++i) {
		Scalar u = inc * i;
		for (size_t j = 0; j <= degree-i; ++j) {
			uvs.push_back({ u, inc*j });
		}
	}

	return uvs;
}

FaceHandle DecimationParametrization::findTargetFace(
	Vec2 &uv, Point &faceBary,
	std::unordered_map<FaceHandle, NGonFace> &faces
) {
	//std::cerr << __FUNCTION__ << " with point " << point << '\n';

	Point bary;

	Scalar positive = -0.001; //std::nextafter(std::nextafter(0., -1.), -1.);

	Vec2::value_type w;
	for (auto &pair : faces) {
		bary = bary2D(uv, pair.second);
		//std::cerr << "\tcalculated bary coords " << bary << '\n';
		if (std::isgreaterequal(bary[0], positive) &&
			std::isgreaterequal(bary[1], positive) &&
			std::isgreaterequal(bary[2], positive)
		) {
			//std::cerr << "\tfound face " << pair.first << '\n';
			faceBary[0] = std::min(1., std::max(0., bary[0]));
			faceBary[1] = std::min(1., std::max(0., bary[1]));
			faceBary[2] = std::min(1., std::max(0., bary[2]));
			return pair.first;
		}
	}

	std::cerr << "ERROR: could not find target face in decimation parametrization for point ";
	std::cerr << uv << '\n';

	return FaceHandle();
}

} // namespace betri
