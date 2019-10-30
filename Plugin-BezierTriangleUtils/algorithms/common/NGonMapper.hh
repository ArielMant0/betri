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

private:

	void mapTriangle(std::vector<Path*> &paths);

	bool checkReverse(bool last, ID f0, ID b0, ID f1, ID b1)
	{
		return (b0 == f1) ? last : !last;
	}
};

}
