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

	void degree(size_t degree) { m_degree = degree; }
	size_t degree() const { return m_degree; }

	bool solve();

	bool solveLocal(const FaceHandle face);

private:

	void prepare();
	void cleanup();

	TriToVertex& ttv(FaceHandle fh) { return m_ctrl.property(m_ttv, fh); }
	VertexToTri& vtt(VertexHandle vh) { return m_mesh.property(m_vtt, vh); }
	Vec2& hmap (VertexHandle vh) { return vtt(vh).uv; }
	int& sysid (VertexHandle _vh) { return m_mesh.property(m_sysid, _vh); }

	Scalar calcCoeffs(VertexHandle vh, int i, int j)
	{
		assert(std::islessequal(hmap(vh)[0], 1.0));
		assert(std::islessequal(hmap(vh)[1], 1.0));
		return eval(i, j, hmap(vh)[0], hmap(vh)[1], m_degree);
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
