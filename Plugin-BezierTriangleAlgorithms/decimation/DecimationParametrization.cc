#include "DecimationParametrization.hh"

#include <random>

namespace betri
{

void DecimationParametrization::prepare()
{
	m_sampleUVs = getRandomUVs(m_samples, true);

#ifdef BEZIER_DEBUG
	std::cerr << "calculated sample uvs:\n";
	for (Vec2 &uv : m_sampleUVs) {
		std::cerr << '\t' << uv << '\n';
	}
#endif
}

void DecimationParametrization::cleanup() {}

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

	std::unordered_map<FaceHandle, NGonFace> faces;
	std::unordered_map<FaceHandle, NGonFace> facesOrig;

	std::unordered_map<VertexHandle, Vec2> vToUV;

	////////////////////////////////////////////////
	// Calculate (u,v) coordinates
	////////////////////////////////////////////////
	{
		Scalar length(0.);

		// accumulate perimeter length of 1-ring
		Point prev = m_mesh.point(*m_mesh.cvv_ccwbegin(from));

		for (auto v_it = m_mesh.cvv_ccwbegin(from); v_it != m_mesh.cvv_ccwend(from); ++v_it) {
			Point tmp = m_mesh.point(*v_it);
			length += (prev - tmp).norm();
			prev = tmp;
		}
		length += (prev - m_mesh.point(*m_mesh.cvv_ccwbegin(from))).norm();

		Scalar lens(0.);

		Point midPoint = m_mesh.point(from);

		Scalar acc(0.);
		std::unordered_map<EdgeHandle, Scalar> cotans;

		prev = m_mesh.point(*m_mesh.cvv_ccwbegin(from));
		auto h_it = m_mesh.cvoh_begin(from);
		// set (u,v) coordinates for all boundary vertices
		for (auto v_it = m_mesh.cvv_ccwbegin(from); v_it != m_mesh.cvv_ccwend(from); ++v_it, ++h_it) {

			Point tmp = m_mesh.point(*v_it);
			lens += (prev - tmp).norm();
			prev = tmp;

			{
				Scalar cotanWeight = 0.0;

				// Compute cotangent edge weights

				HalfedgeHandle h0 = *h_it;
				VertexHandle v0 = m_mesh.to_vertex_handle(h0);
				Point p0 = m_mesh.point(v0);

				HalfedgeHandle h1 = m_mesh.opposite_halfedge_handle(h0);
				Point p1 = midPoint;

				HalfedgeHandle h2 = m_mesh.next_halfedge_handle(h0);
				Point p2 = m_mesh.point(m_mesh.to_vertex_handle(h2));
				Point d0 = (p0 - p2).normalize();
				Point d1 = (p1 - p2).normalize();
				cotanWeight += 1.0 / tan(acos(d0 | d1));

				h2 = m_mesh.next_halfedge_handle(h1);
				p2 = m_mesh.point(m_mesh.to_vertex_handle(h2));
				d0 = (p0 - p2).normalize();
				d1 = (p1 - p2).normalize();
				cotanWeight += 1.0 / tan(acos(d0 | d1));

				acc += cotanWeight;
				cotans.insert({ m_mesh.edge_handle(h0), cotanWeight });
			}

			// get angle for this vertex (normalized contangent weight
			Scalar angle = lens * (1.0 / length) * (M_PI * 2.0);

			vToUV.insert({ *v_it, NGonMapper<Vec2>::cornerFromAngle(angle) });
		}

		Vec2 average(0.);

		// set weighted position for interior vertex
		for (auto hh_it = m_mesh.cvoh_begin(from); hh_it != m_mesh.cvoh_end(from); ++hh_it) {
			// cotan weights
			average += vToUV[m_mesh.to_vertex_handle(*hh_it)] * cotans[m_mesh.edge_handle(*hh_it)];
		}
		vToUV.insert({ from, average * (1.0 / acc) });
	}


	const auto area = [&](NGonFace &p) {
		Scalar a = (p[1] - p[0]).norm();
		Scalar b = (p[2] - p[0]).norm();
		Scalar c = (p[2] - p[1]).norm();

		Scalar s = (a + b + c) / 2.;

		return std::sqrt(s*(s-a)*(s-b)*(s-c));
	};

	unsigned int degree = m_mesh.degree();
	unsigned int cpNum = pointsFromDegree(degree);

	////////////////////////////////////////////////
	// Create blue/green triangles
	////////////////////////////////////////////////
	for (auto v_it = m_mesh.cvv_ccwbegin(from); v_it != m_mesh.cvv_ccwend(from); ++v_it) {

		auto next = std::next(v_it);
		if (next == m_mesh.cvv_ccwend(from)) {
			next = m_mesh.cvv_ccwbegin(from);
		}

		VertexHandle v0 = *v_it;
		VertexHandle v1 = *next;

		HalfedgeHandle hh = m_mesh.find_halfedge(from, v0);
		FaceHandle f = m_mesh.face_handle(hh);
		if (!f.is_valid() || !m_mesh.adjToVertex(f, v1)) {
			f = m_mesh.opposite_face_handle(hh);
		}

		if (!m_mesh.adjToVertex(f, v1)) return false;

		int c0 = m_mesh.cpCornerIndex(f, v0);
		if (c0 < 0) return false;

		int c1 = m_mesh.cpCornerIndex(f, v1);
		if (c1 < 0) return false;

		// correct order
		VertexHandle v00 = c0 == 0 ? v0 : (c1 == 0 ? v1 : to);
		VertexHandle v01 = c0 == 1 ? v0 : (c1 == 1 ? v1 : to);
		VertexHandle v02 = c0 == 2 ? v0 : (c1 == 2 ? v1 : to);

		if (v0 != to && v1 != to) {
			assert(f != f0 && f != f1);
			// the face after the halfedge collapse
			faces.insert({ f, { vToUV[v00], vToUV[v01], vToUV[v02] } });
			assert(std::isgreater(area(faces[f]), 0.));
		}

		v00 = c0 == 0 ? v0 : (c1 == 0 ? v1 : from);
		v01 = c0 == 1 ? v0 : (c1 == 1 ? v1 : from);
		v02 = c0 == 2 ? v0 : (c1 == 2 ? v1 : from);

		facesOrig.insert({ f, { vToUV[v00], vToUV[v01], vToUV[v02] } });

		// if degenerate triangle -> abort
		if (!std::isgreater(area(facesOrig[f]), 0.)) return false;
	}

	// if somehow a face was lost -> abort
	if (faces.size() != n - 2) return false;

	Vec2 checkTo0 = vToUV[to];
	Point checkTo1 = m_mesh.point(to);

	////////////////////////////////////////////////
	// Sample triangles
	////////////////////////////////////////////////
	for (auto &pair : faces) {

		// only calculate for remaining faces

		FitCollection fit;
		fit.face = pair.first;

		Point faceBary;

		for (Vec2 &uv : m_sampleUVs) {
			Vec2 facePos = trianglePoint(uv, pair.second);
			// find the non-collapsed face in which this uv point lies
			FaceHandle target = findTargetFace(facePos, faceBary, facesOrig);
			// if no target could be found -> abort
			if (!target.is_valid()) return false;
			// sample the target face at its uv position
			fit.add(evalSurface(m_mesh.data(target).points(), faceBary, degree));
		}

		fitColl.push_back(fit);
	}

	return true;
}

std::vector<Vec2> DecimationParametrization::getSampleUVs(const size_t degree)
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
std::vector<Vec2> DecimationParametrization::getRandomUVs(const size_t n, bool sampleUniform)
{
	std::vector<Vec2> uvs;

	if (sampleUniform) {
		uvs = getSampleUVs(5);
	}

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0.0, 1.0);

	std::array<double, 2> tmp;
	while (uvs.size() < n) {

		// generate three random values [0,1]
		tmp[0] = dis(gen);
		tmp[1] = dis(gen);
		tmp[2] = dis(gen);
		std::sort(tmp.begin(), tmp.end());

		// construct barycentric coordinate
		Vec2 bary(tmp[1]-tmp[0], 1.0-tmp[1]);

		if (!std::any_of(uvs.begin(), uvs.end(), [&](const Vec2 &vec) {
			return (vec - bary).norm() < 0.0001;
		})) {
			uvs.push_back(std::move(bary));
		} else {
			// see if shifting the coordinate a little is already enough
			tmp[0] = dis(gen);
			tmp[1] = dis(gen);
			// "sort"
			if (tmp[0] < tmp[1]) {
				auto swap = tmp[0];
				tmp[0] = tmp[1];
				tmp[1] = swap;
			}

			bary += Vec2(1.0 - tmp[1], tmp[1] - tmp[0]) * 0.01;

			if (!std::any_of(uvs.begin(), uvs.end(), [&](const Vec2 &vec) {
				return (vec - bary).norm() < 0.0001;
			})) {
				uvs.push_back(std::move(bary));
			}
		}

		assert(std::islessequal(bary[0], 1.0));
		assert(std::islessequal(bary[1], 1.0));
		assert(std::isgreaterequal(bary[0], 0.0));
		assert(std::isgreaterequal(bary[1], 0.0));
		assert(std::islessequal(bary[0] + bary[1], 1.0));

	}
	assert(uvs.size() == n);

	return uvs;
}

FaceHandle DecimationParametrization::findTargetFace(
	const Vec2 &point, Point &faceBary,
	const std::unordered_map<FaceHandle, NGonFace> &faces
) {
	Point bary;

	Scalar positive = -0.00001;

	for (auto &pair : faces) {
		bary = bary2D(point, pair.second);

		// if barycentric coords are all positive, we found the right triangle
		if (std::isgreater(bary[0], positive) &&
			std::isgreater(bary[1], positive) &&
			std::isgreater(bary[2], positive)
		) {
			// make sure we not just close to 0 or 1
			faceBary[0] =  std::min(1., std::max(0., bary[0]));
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
