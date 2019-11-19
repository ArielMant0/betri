#include "ShortestPath.hh"

//struct PathHash
//{
//	std::size_t operator()(const ShortestPath& s) const
//	{
//		return std::hash<ID>{}(s.first());
//	}
//};

// stores the shortest path for each pair of tiles that is needed to
// construct the delaunay triangulation
static std::unordered_set<betri::ShortestPath> s_paths;

namespace betri
{

bool ShortestPath::replace(
	const BezierTMesh &mesh,
	const VH adj,
	const VH now
) {
	// find the two edges adj to the old vertex
	for (size_t i = 0; i < m_border.size(); ++i) {
		if (m_border[i] == adj) {
			m_border[i] = now;
			return true;
		}
	}

	return false;
}

const ShortestPath& ShortestPath::path(ID id1, ID id2)
{
	auto p = s_paths.find({ id1, id2 });
	assert(p != s_paths.end());
	return *p;
}

void ShortestPath::path(ShortestPath &path)
{
	s_paths.insert(path);
}

std::unordered_set<betri::ShortestPath>* ShortestPath::pathList()
{
	return &s_paths;
}

bool ShortestPath::has(const ID id1, const ID id2)
{
	return s_paths.find({ id1, id2 }) != s_paths.end();
}

void ShortestPath::connect(BezierTMesh &mesh, const ID id1, const ID id2, const ID id3)
{

}

void ShortestPath::replace(
	const BezierTMesh &mesh,
	const VH adj,
	const VH now,
	const ID hint1,
	const ID hint2
) {
	for (auto path : s_paths) {
		// TODO: make better
		if ((path.partOf(hint1) || path.partOf(hint2))) {
			if (path.replace(mesh, adj, now)) return;
		}
	}
}

void ShortestPath::clear()
{
	s_paths.clear();
}

}