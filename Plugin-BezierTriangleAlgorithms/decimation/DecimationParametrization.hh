#pragma once

#include "../common/Parametrization.hh"
#include "../common/NGonMapper.hh"

#include <ACG/Math/VectorT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <unordered_map>

namespace betri
{

class DecimationParametrization : public Parametrization
{
public:

	using NGonFace = std::array<Vec2, 3>;

	// TODO: need other handle?
	explicit DecimationParametrization(BezierTMesh &mesh) :
		Parametrization(mesh), m_mapper(mesh, OpenMesh::VPropHandleT<Vec2>())
	{
		prepare();
	}

	~DecimationParametrization()
	{
		cleanup();
	}

	// directly solve parametrization
	bool solve() override;

    // add/remove needed properties for weights, texture coords etc.
	void prepare() override;
	void cleanup() override;

	bool solveLocal(
		const VertexHandle from,
		const VertexHandle to,
		std::vector<FitCollection> &fitColl,
		const bool print
	);

	std::vector<Vec2>& getBarycentricCoords()
	{
		return m_sampleUVs;
	}

	static bool test(BezierTMesh *mesh=nullptr);

private:

	static std::vector<Vec2> getSampleUVs(size_t degree);
	static std::vector<Vec2> getRandomUVs(size_t n, bool sampleUniform=false);

	static Vec3 bary2D(const Vec2 &uv, const NGonFace &corners)
	{
		//std::cerr << "corners:\n\t" << corners[0] << "\n\t" << corners[1];
		//std::cerr << "\n\t" << corners[2] << "\npoint: " << uv << std::endl;

		Vec2 ba = corners[1] - corners[0];
		Vec2 ca = corners[2] - corners[0];
		Vec2 pa = uv - corners[0];

		Scalar denom = ba[0] * ca[1] - ba[1] * ca[0];

		if (1.0 + fabs(denom) == 1.0) {
			return Vec3(-1., -1, -1.);
		}

		Vec3 result(
			0.,
			1.0 + ((pa[0] * ca[1] - pa[1] * ca[0]) / denom) - 1.0,
			1.0 + ((ba[0] * pa[1] - ba[1] * pa[0]) / denom) - 1.0
		);
		result[0] = 1. - result[1] - result[2];

		return result;
	}

	static Vec2 trianglePoint(const Vec2 &uv, const NGonFace &points)
	{
		Scalar w = 1. - uv[0] - uv[1];

		//std::cerr << __FUNCTION__ << "\n\tuv " << uv << " " << w;
		//std::cerr << "\n\t pos " << points[0] << " " << points[1] << " " << points[2] << "\n";

		return points[0] * uv[0] + points[1] * uv[1] + points[2] * w;
	}

	static FaceHandle findTargetFace(
		const Vec2 &uv,
		Point &faceBary,
		const std::unordered_map<FaceHandle, NGonFace> &faces
	);

	///////////////////////////////////////////////////////////
	// member variables
	///////////////////////////////////////////////////////////

	std::vector<Vec2> m_sampleUVs;

	/// maps boundary vertices (to some convex polygon dictated by the mapper class)
	NGonMapper<Vec2> m_mapper;
};

}

