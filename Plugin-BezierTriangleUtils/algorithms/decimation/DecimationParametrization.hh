#pragma once

#include "../common/Parametrization.hh"
#include "../common/NGonMapper.hh"

#include <ACG/Math/VectorT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

namespace betri
{

class DecimationParametrization : public Parametrization
{
public:

	using NGonFace = std::array<Vec2, 3>;

	explicit DecimationParametrization(BezierTMesh &mesh, OpenMesh::VPropHandleT<Vec2> &uvProp) :
		Parametrization(mesh), m_mapper(mesh, uvProp)
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

	static bool test(BezierTMesh *mesh=nullptr);

	static std::vector<Vec2> getSampleUVs(size_t degree);

	static Vec3 bary2D(Vec2 &uv, NGonFace &corners)
	{
		Vec2 v0 = corners[1] - corners[0];
		Vec2 v1 = corners[2] - corners[0];
		Vec2 v2 = uv - corners[0];

		Vec2::value_type d00 = v0 | v0;
		Vec2::value_type d01 = v0 | v1;
		Vec2::value_type d11 = v1 | v1;
		Vec2::value_type d20 = v2 | v0;
		Vec2::value_type d21 = v2 | v1;
		Vec2::value_type denom = d00 * d11 - d01 * d01;

		// https://gamedev.stackexchange.com/a/23745
		// TODO: bary 2d correct u,v,w ?
		Vec2::value_type v = (d11 * d20 - d01 * d21) / denom;
		Vec2::value_type w = (d00 * d21 - d01 * d20) / denom;
		Vec2::value_type u = 1.0 - v - w;

		return Vec3(u, v, w);
	}

private:

	Vec2 trianglePoint(Vec2 uv, NGonFace points)
	{
		Scalar w = 1. - uv[0] - uv[1];

		//std::cerr << __FUNCTION__ << "\n\tuv " << uv << " " << w;
		//std::cerr << "\n\t pos " << points[0] << " " << points[1] << " " << points[2] << "\n";

		return points[0] * uv[0] + points[1] * uv[1] + points[2] * w;
	}

	FaceHandle findTargetFace(
		Vec2 &uv,
		Point &faceBary,
		std::unordered_map<FaceHandle, NGonFace> &faces
	);

	///////////////////////////////////////////////////////////
	// member variables
	///////////////////////////////////////////////////////////

	std::vector<Vec2> m_sampleUVs;

	/// maps boundary vertices (to some convex polygon dictated by the mapper class)
	NGonMapper<Vec2> m_mapper;
};

}

