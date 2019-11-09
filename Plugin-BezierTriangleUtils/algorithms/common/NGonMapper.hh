#pragma once

#include "BoundaryMapper.hh"

#include <OpenMesh/Core/Utils/Property.hh>

namespace betri
{

class NGonMapper : public BoundaryMapper
{
public:

	explicit NGonMapper(BezierTMesh &mesh, OpenMesh::VPropHandleT<VertexToTri> &vtt) :
		BoundaryMapper(mesh, vtt) {}

    void map(std::vector<Path*> &paths) override;

	Vec2 middle() override { return Vec2(0.33f, 0.33f); }

private:

	void mapTriangle(std::vector<Path*> &paths);

};

}
