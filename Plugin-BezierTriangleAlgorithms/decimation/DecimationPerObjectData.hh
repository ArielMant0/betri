#pragma once

#include "Decimation.hh"

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenFlipper/common/perObjectData.hh>

class DecimationPerObjectData : public PerObjectData
{

public:

	DecimationPerObjectData(BezierTMesh& mesh) : m_decimate(mesh) {}

	virtual ~DecimationPerObjectData() {}

	betri::Decimation& decimator() { return m_decimate; }

private:
	// create an decimation object
	betri::Decimation m_decimate;
};
