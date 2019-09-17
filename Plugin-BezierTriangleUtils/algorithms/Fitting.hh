#pragma once

#include "Common.hh"

#include <ACG/Math/VectorT.hh>
#include <OpenMesh/Core/Utils/Property.hh>

#include <Eigen/Dense>
//#include <Eigen/SparseCore>

namespace betri
{

// for a fixed number of control points (dictated by degree)
// || t-Matrix X coeff-Matrix X controlPoint-Matrix - surfacePoint-Matrix ||² = 0

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
		OpenMesh::VPropHandleT<VertexToTri> &vtt
	) :
		m_mesh(m),
		m_ctrl(c),
		m_ttv(ttv),
		m_vtt(vtt),
		m_degree(0),
		m_cpNum(0)
	{
		prepare();
	}

	void solve();

	TriToVertex& ttv(FaceHandle fh) { return m_ctrl.property(m_ttv, fh); }
	VertexToTri& vtt(VertexHandle vh) { return m_mesh.property(m_vtt, vh); }
	Vec2& hmap (VertexHandle vh) { return vtt(vh).uv; }
	int& sysid (VertexHandle _vh) { return m_mesh.property(m_sysid, _vh); }

private:

	void prepare();
	void cleanup();

	size_t calcCPCount(unsigned int degree);

	void solveLocal(Vertices &inner, const FaceHandle face);

	Scalar calcCoeffs(VertexHandle vh, int i, int j)
	{
		//assert(vtt(vh).uv.norm() <= 1.0);
		// TODO: ?!
		return eval(i, j, vtt(vh).uv[0], vtt(vh).uv[1], m_degree);
	}

	///////////////////////////////////////////////////////////
	// member variables
	///////////////////////////////////////////////////////////

	size_t m_degree, m_cpNum;
	BezierTMesh &m_mesh, &m_ctrl;

	OpenMesh::FPropHandleT<TriToVertex>	&m_ttv;
	OpenMesh::VPropHandleT<VertexToTri>	&m_vtt;
	OpenMesh::VPropHandleT<int> m_sysid;
};

}