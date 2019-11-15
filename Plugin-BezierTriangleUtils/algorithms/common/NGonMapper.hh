#pragma once

#include "BoundaryMapper.hh"

#include <OpenMesh/Core/Utils/Property.hh>

namespace betri
{

template <typename T>
class NGonMapper : public BoundaryMapper<T>
{
public:

	explicit NGonMapper(BezierTMesh &mesh, OpenMesh::VPropHandleT<T> &uvProp) :
		BoundaryMapper(mesh, uvProp) {}

    void map(std::vector<Path*> &paths) override;

	Vec2 middle() override { return Vec2(0.33f, 0.33f); }
	Vec2 middle(size_t n)
	{
		switch (n) {
			default:
			case 3: return Vec2(0.33f, 0.33f);
		}
	}

private:

	void mapTriangle(std::vector<Path*> &paths);

};

}
