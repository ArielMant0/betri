#pragma once

#include "Common.hh"

#include <ACG/Math/VectorT.hh>
#include <OpenMesh/Core/Utils/Property.hh>

#include <Eigen/Dense>
//#include <Eigen/SparseCore>

namespace betri
{

class Fitting
{
	using EigenMatT = Eigen::MatrixXd;
	using EigenVectorT = Eigen::VectorXd;

	using Vertices = std::vector<VertexHandle>;

public:

	Fitting(
		BezierTMesh &m,
		BezierTMesh &c,
		OpenMesh::FPropHandleT<TriToVertex> &ttv,
		OpenMesh::VPropHandleT<VertexToTri> &vtt,
		size_t sampleCount=0
	) :
		m_mesh(m),
		m_ctrl(c),
		m_ttv(ttv),
		m_vtt(vtt),
		m_degree(0),
		m_cpNum(0)
	{
		// make sure we sample at least as many points on the surface as there are control points
		m_samples = std::max(sampleCount, pointsFromDegree(m.degree()));
		prepare();
	}

	void degree(size_t degree) { m_degree = degree; }
	size_t degree() const { return m_degree; }

	bool solve();

	bool solveLocal(const FaceHandle face);

	static bool test(BezierTMesh *mesh=nullptr);

private:

	void prepare();
	void cleanup();

	TriToVertex& ttv(FaceHandle fh) { return m_ctrl.property(m_ttv, fh); }
	VertexToTri& vtt(VertexHandle vh) { return m_mesh.property(m_vtt, vh); }
	Vec2& hmap (VertexHandle vh) { return vtt(vh).uv; }
	int& sysid (VertexHandle _vh) { return m_mesh.property(m_sysid, _vh); }

	static Scalar calcCoeffs(Vec2 uv, int i, int j, size_t degree)
	{
		assert(std::islessequal(uv[0], 1.0));
		assert(std::islessequal(uv[1], 1.0));
		return eval(i, j, uv[0], uv[1], degree);
	}

	static bool solveSystem(EigenMatT &A, EigenVectorT &rhs, EigenVectorT &result);

	///////////////////////////////////////////////////////////
	// member variables
	///////////////////////////////////////////////////////////

	size_t m_degree, m_cpNum, m_samples;
	BezierTMesh &m_mesh, &m_ctrl;

	OpenMesh::FPropHandleT<TriToVertex>	&m_ttv;
	OpenMesh::VPropHandleT<VertexToTri>	&m_vtt;
	OpenMesh::VPropHandleT<int> m_sysid;
};

}
