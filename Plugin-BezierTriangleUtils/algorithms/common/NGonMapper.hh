#pragma once

#include "BoundaryMapper.hh"

#include <OpenMesh/Core/Utils/Property.hh>

namespace betri
{

class NGonMapper : public BoundaryMapper
{
public:

	explicit NGonMapper(
		BezierTMesh &mesh,
		OpenMesh::VPropHandleT<VertexToTri> &vtt,
		OpenMesh::VPropHandleT<ID> &id
	) : BoundaryMapper(mesh, vtt, id) {}

    void map(std::vector<Path*> &paths) override;

};

}
