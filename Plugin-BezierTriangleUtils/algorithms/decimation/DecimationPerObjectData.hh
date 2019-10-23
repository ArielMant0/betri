#pragma once

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenFlipper/common/perObjectData.hh>

class DecimationPerObjectData : public PerObjectData
{

public:

	DecimationPerObjectData(BezierTMesh& mesh, BezierTMesh &ctrl) : m_decimate(mesh, ctrl) {}

	virtual ~DecimationPerObjectData() {}

	betri::Decimation& remesher() { return m_decimate; }

private:
	// create an decimation object
	betri::Decimation m_decimate;
};
