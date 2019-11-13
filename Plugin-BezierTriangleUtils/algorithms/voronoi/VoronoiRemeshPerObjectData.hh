#pragma once

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenFlipper/common/perObjectData.hh>
#include "VoronoiRemesh.hh"

class VoronoiRemeshPerObjectData : public PerObjectData
{

public:

	VoronoiRemeshPerObjectData(
		BezierTMesh& mesh,
		BezierTMesh &ctrl,
		size_t count
	) : m_remesher(mesh, ctrl, count) {}

	virtual ~VoronoiRemeshPerObjectData() {}

	betri::VoronoiRemesh& remesher() { return m_remesher; }

private:
	// create an voronoi remesh object
	betri::VoronoiRemesh m_remesher;
};