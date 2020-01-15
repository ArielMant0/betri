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

	// typedefs
	using NGonFace = std::array<Vec2, 3>;

	explicit DecimationParametrization(BezierTMesh &mesh) :
		Parametrization(mesh),
		m_mapper(mesh, OpenMesh::VPropHandleT<Vec2>()),
		m_samples(40u) // default that will be overriden by plugin
	{}

	~DecimationParametrization()
	{
		cleanup();
	}

	// directly solve parametrization
	bool solve() override;

    // add/remove needed properties for weights, texture coords etc.
	void prepare() override;
	void cleanup() override;

	void samples(size_t number) { m_samples = number; }
	size_t samples() const { return m_samples; }

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

private:

	static std::vector<Vec2> getSampleUVs(const size_t degree);
	static std::vector<Vec2> getRandomUVs(const size_t n, bool sampleUniform=false);

	static Vec3 bary2D(const Vec2 &uv, const NGonFace &corners)
	{
		Vec2 ba = corners[1] - corners[0];
		Vec2 ca = corners[2] - corners[0];
		Vec2 pa = uv - corners[0];

		Scalar denom = ba[0] * ca[1] - ba[1] * ca[0];

		if (1.0 + fabs(denom) == 1.0) {
			return Vec3(-1., -1, -1.);
		}

		// use +1-1 for accuracy
		Vec3 result(
			0.,
			1.0 + ((pa[0] * ca[1] - pa[1] * ca[0]) / denom) - 1.0,
			1.0 + ((ba[0] * pa[1] - ba[1] * pa[0]) / denom) - 1.0
		);
		result[0] = 1. - result[1] - result[2];

		return result;
	}

	static inline Vec2 trianglePoint(const Vec2 &uv, const NGonFace &points)
	{
		Scalar w = 1. - uv[0] - uv[1];
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

	size_t m_samples;
};

}

