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

ShortestPath ShortestPath::path(ID id1, ID id2)
{
	auto p = s_paths.find({ id1, id2 });
	assert(p != s_paths.end());
	return *p;
}

void ShortestPath::path(ShortestPath &path)
{
	s_paths.insert(path);
}

bool ShortestPath::has(const ID id1, const ID id2)
{
	return s_paths.find({ id1, id2 }) != s_paths.end();
}

void ShortestPath::replace(const EH toReplace, const EH with)
{
	for (auto path : s_paths) {
		// TODO: make better
		if (path.contains(toReplace)) {
			std::replace(path.edges().begin(), path.edges().end(), toReplace, with);
			return;
		}
	}
}

void ShortestPath::clear()
{
	s_paths.clear();
}

}