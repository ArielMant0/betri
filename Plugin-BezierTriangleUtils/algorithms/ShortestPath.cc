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
	const VH now,
	std::function<void(EH)> funcOld,
	std::function<void(EH, EH)> funcNew
) {
	HH from, to;
	VH vh;
	int adjCount = 0;

	// find the two edges adj to the old vertex
	for (size_t i = 0; i < m_border.size() && adjCount < 2; ++i) {
		to = mesh.halfedge_handle(m_border[i], 0);
		from = mesh.halfedge_handle(m_border[i], 1);

		if (mesh.to_vertex_handle(from) == adj || mesh.to_vertex_handle(to) == adj) {
			// other vertex the edge is connected to
			vh = mesh.to_vertex_handle(from) == adj ?
				mesh.to_vertex_handle(to) :
				mesh.to_vertex_handle(from);

			adjCount++;

			auto old = m_border[i];

			// replace edge such that it is adj to the new vertex
			for (auto tmp = mesh.cvoh_begin(vh); tmp != mesh.cvoh_end(vh); ++tmp) {
				if (mesh.to_vertex_handle(*tmp) == now) {
					m_border[i] = mesh.edge_handle(*tmp);
					break;
				}
			}

			funcNew(m_border[i], old);
			funcOld(old);
		}
	}

	return adjCount == 2;
}

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

void ShortestPath::replace(
	const BezierTMesh &mesh,
	const VH adj,
	const VH now,
	const ID hint1,
	const ID hint2,
	std::function<void(EH)> funcOld,
	std::function<void(EH, EH)> funcNew
) {
	for (auto path : s_paths) {
		// TODO: make better
		if ((path.partOf(hint1) || path.partOf(hint2))) {
			if (path.replace(mesh, adj, now, funcOld, funcNew)) return;
		}
	}
}

void ShortestPath::clear()
{
	s_paths.clear();
}

}