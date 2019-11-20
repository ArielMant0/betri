#pragma once

#include "../common/Fitting.hh"

#include <OpenMesh/Core/Utils/Property.hh>

namespace betri
{

class VoronoiFitting : public Fitting
{
public:

    VoronoiFitting() = delete;

	VoronoiFitting(
		BezierTMesh &m,
		BezierTMesh &c,
		OpenMesh::FPropHandleT<TriToVertex> &ttv,
		OpenMesh::VPropHandleT<VertexToTri> &vtt,
		size_t sampleCount=0
	) :
        Fitting(m),
		m_ctrl(c),
		m_ttv(ttv),
		m_vtt(vtt),
		m_cpNum(0)
	{
		// make sure we sample at least as many points on the surface as there are control points
		m_samples = std::max(sampleCount, (size_t)30);
		prepare();
	}

	bool solve() override;

	void prepare() override;
	void cleanup() override;

	bool solveLocal(const FaceHandle face);

	static bool test(BezierTMesh *mesh=nullptr);

private:

	void sortInner(const FaceHandle face);

	TriToVertex& ttv(FaceHandle fh) { return m_ctrl.property(m_ttv, fh); }
	VertexToTri& vtt(VertexHandle vh) { return m_mesh.property(m_vtt, vh); }
	Vec2& hmap (VertexHandle vh) { return vtt(vh).uv; }
	int& sysid (VertexHandle _vh) { return m_mesh.property(m_sysid, _vh); }


	///////////////////////////////////////////////////////////
	// member variables
	///////////////////////////////////////////////////////////

	size_t m_cpNum, m_samples;
	BezierTMesh &m_ctrl;

	OpenMesh::FPropHandleT<TriToVertex>	&m_ttv;
	OpenMesh::VPropHandleT<VertexToTri>	&m_vtt;
	OpenMesh::VPropHandleT<int> m_sysid;
};

}
