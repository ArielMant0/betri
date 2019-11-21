#include "DecimationParametrization.hh"

#include <random>

namespace betri
{

void DecimationParametrization::prepare()
{
	// note: we take the degree+1 so we always sample inside the triangle
	//m_sampleUVs = getSampleUVs(m_mesh.degree()+2);
	m_sampleUVs = getRandomUVs(40);

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
	for (auto h_it = m_mesh.cvoh_ccwbegin(from); h_it != m_mesh.cvoh_ccwend(from); ++h_it, ++idx) {
		if (m_mesh.to_vertex_handle(*h_it) == to) {
			toCorner = m_mapper.corner(idx, n);
			break;
		}
	}

	const auto area = [&](NGonFace &p) {
		Scalar a = (p[1] - p[0]).norm();
		Scalar b = (p[2] - p[0]).norm();
		Scalar c = (p[2] - p[1]).norm();

		Scalar s = (a + b + c) / 2.;

		return std::sqrt(s*(s-a)*(s-b)*(s-c));
	};

	Vec2 middle = m_mapper.middle(n);

	idx = 0;
	// calculate uv corners for each triangle *after* collapse
	for (auto h_it = m_mesh.cvoh_ccwbegin(from); h_it != m_mesh.cvoh_ccwend(from); ++h_it, ++idx) {

		auto next = std::next(h_it);
		if (next == m_mesh.cvoh_ccwend(from)) {
			next = m_mesh.cvoh_ccwbegin(from);
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
				m_mapper.corner((idx + 1) % n, n),
				toCorner
			}};
			faces.insert({ f, corners });
			facesOrig.insert({ f, {
				corners[0],
				corners[1],
				middle
			} });

			assert(std::isgreater(area(corners), 0.));
			assert(std::isgreater(area(facesOrig[f]), 0.));

		} else {
			facesOrig.insert({ f, {
				m_mapper.corner(idx, n),
				m_mapper.corner((idx + 1) % n, n),
				middle
			} });
		}
	}
	assert(faces.size() == n - 2);

	//if (print) {
	//	for (auto &stuff : facesOrig) {
	//		std::cerr << "face " << stuff.first << "\n\toriginal (blue):\n";
	//		for (auto &pair : stuff.second) {
	//			std::cerr << "\t\t" << pair << "\n";
	//		}

	//		if (stuff.first != f0 && stuff.first != f1) {
	//			auto &other = faces[stuff.first];
	//			std::cerr << "\tnew (green):\n";
	//			for (auto &pair : other) {
	//				std::cerr << "\t\t" << pair << "\n";
	//			}
	//		}
	//	}
	//}

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
			fit.add(evalSurface(m_mesh.data(target).points(), faceBary, degree));
			/*if (print) {
				std::cerr << "adding surface sample for target "<<  target << '\n';
				std::cerr << "\tuv " << uv << '\n';
				std::cerr << "\tface uv " << faceUV << '\n';
				std::cerr << "\tposition " << fit.points.back() << '\n';
			}*/
		}

		fitColl.push_back(fit);
	}

	return true;
}

bool DecimationParametrization::test(BezierTMesh *mesh)
{
	std::unordered_map<FaceHandle, NGonFace> faces, facesOrig;

	assert(mesh != nullptr);

	VertexHandle a = mesh->add_vertex(Point(1., 0.5, 0.)); // A
	VertexHandle b = mesh->add_vertex(Point(0.654508, 0.975528, 0.)); // B - to collapse into
	VertexHandle c = mesh->add_vertex(Point(0.0954915, 0.792893, 0.)); // C
	VertexHandle d = mesh->add_vertex(Point(0.0954915, 0.206107, 0.)); // D
	VertexHandle e = mesh->add_vertex(Point(0.654508, 0.0244717, 0.)); // E
	VertexHandle f = mesh->add_vertex(Point(0.5, 0.5, 0.)); // F - middle

	FaceHandle abf = mesh->add_face(a, b, f, true);
	FaceHandle bcf = mesh->add_face(b, c, f, true);
	FaceHandle cdf = mesh->add_face(c, d, f, true);
	FaceHandle def = mesh->add_face(d, e, f, true);
	FaceHandle eaf = mesh->add_face(e, a, f, true);

	Point add(0., 0., 0.5);

	size_t degree = mesh->degree();

	for (FaceHandle face : mesh->faces()) {
		auto it = mesh->cfv_begin(face);
		Point p0 = mesh->point(*it++);
		Point p1 = mesh->point(*it++);
		Point p2 = mesh->point(*it++);
		facesOrig.insert({ face, { Vec2(p0[0], p0[1]), Vec2(p1[0], p1[1]), Vec2(p2[0], p2[1]) } });

		auto &cp = mesh->data(face);
		cp.controlPoint(1, cp.controlPoint(1) + add);
		cp.controlPoint(3, cp.controlPoint(3) + add);
		cp.controlPoint(4, cp.controlPoint(4) + add);
	}

	faces.insert({ cdf, {
		Vec2(0.0954915, 0.792893),
		Vec2(0.0954915, 0.206107),
		Vec2(0.654508, 0.975528)
	} });
	faces.insert({ def, {
		Vec2(0.0954915, 0.206107),
		Vec2(0.654508, 0.0244717),
		Vec2(0.654508, 0.975528)
	} });
	faces.insert({ eaf, {
		Vec2(0.654508, 0.0244717),
		Vec2(1., 0.5),
		Vec2(0.654508, 0.975528)
	} });

	auto sampleUVs = getSampleUVs(degree);

	std::cerr << std::endl;

	for (auto &pair : faces) {

		// only calculate for remaining faces

		Point faceBary;

		for (Vec2 &uv : sampleUVs) {
			std::cerr << "green face " << pair.first << '\n';

			Vec2 facePos = trianglePoint(uv, pair.second);
			// find the non-collapsed face in which this uv point lies
			FaceHandle target = findTargetFace(facePos, faceBary, facesOrig);
			assert(target.is_valid());

			std::cerr << "blue face " << target << '\n';
			// sample the target face at its uv position
			auto surface = evalSurface(mesh->data(target).points(), faceBary, degree);

			std::cerr << "\tgreen uv " << uv << '\n';
			std::cerr << "\tblue uv " << faceBary << '\n';
			std::cerr << "\ttriangle point " << facePos << '\n';
			std::cerr << "\tsurface point " << surface << '\n';

			auto ps = facesOrig[target];
			auto testPoint = trianglePoint(Vec2(faceBary[0], faceBary[1]), ps);
			std::cerr << "\ttest point " << testPoint << '\n';

			std::cerr << "//---------- " << (facePos - testPoint).norm() << " ----------//\n";
			assert((facePos - testPoint).norm() <= 0.00001);
		}
		std::cerr << std::endl << std::endl;
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

// https://cs.stackexchange.com/a/3229
std::vector<Vec2> DecimationParametrization::getRandomUVs(size_t n)
{
	std::vector<Vec2> uvs;
	uvs.reserve(n);

	size_t border = 9;

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0.0, 1.0);

	std::array<double, 3> tmp;
	for (size_t i = 0; i < n-border; ++i) {
		tmp[0] = dis(gen);
		tmp[1] = dis(gen);
		tmp[2] = dis(gen);
		std::sort(tmp.begin(), tmp.end());

		uvs.push_back(Vec2(tmp[0], tmp[2]-tmp[1]));

		Vec2 test = uvs.back();
		assert(std::islessequal(test[0], 1.0));
		assert(std::islessequal(test[1], 1.0));
		assert(std::isgreaterequal(test[0], 0.0));
		assert(std::isgreaterequal(test[1], 0.0));
		assert(std::islessequal(test[0] + test[1], 1.0));

	}

	size_t edgeCount = (border-3) / 3;
	Scalar add = 1. / (edgeCount + 1);
	// always sample border vertices as well
	for (size_t i = 1; i <= edgeCount; ++i) {
		uvs.push_back(Vec2(add * i, 0.));
		uvs.push_back(Vec2(0., add * i));
		uvs.push_back(Vec2(add * i, 1. - add * i));
	}
	uvs.push_back(Vec2(0., 0.));
	uvs.push_back(Vec2(1., 0.));
	uvs.push_back(Vec2(0., 1.));


	return uvs;
}

FaceHandle DecimationParametrization::findTargetFace(
	Vec2 &point, Point &faceBary,
	std::unordered_map<FaceHandle, NGonFace> &faces
) {
	//std::cerr << __FUNCTION__ << " with point " << point << '\n';

	Point bary;

	Scalar positive = -0.001; //std::nextafter(std::nextafter(0., -1.), -1.);

	for (auto &pair : faces) {
		bary = bary2D(point, pair.second);
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
	std::cerr << point << '\n';

	return FaceHandle();
}

} // namespace betri
