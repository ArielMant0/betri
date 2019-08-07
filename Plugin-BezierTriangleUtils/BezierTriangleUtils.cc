#include "BezierTriangleUtils.hh"

#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <queue>
#include <random>

namespace betri
{

void addBezierTriangles(BezierTMesh &btmesh)
{
	for (auto f_it = btmesh.faces_begin(); f_it != btmesh.faces_end(); ++f_it) {
		auto v_it = btmesh.fv_begin(*f_it);
		auto p1 = btmesh.point(*v_it); v_it++;
		auto p2 = btmesh.point(*v_it); v_it++;
		auto p3 = btmesh.point(*v_it);

		btmesh.data(*f_it).addPoint(p1);
		btmesh.data(*f_it).addPoint(p1 * 0.5f + p2 * 0.5f);
		btmesh.data(*f_it).addPoint(p2);
		btmesh.data(*f_it).addPoint(p2 * 0.5f + p3 * 0.5f);
		btmesh.data(*f_it).addPoint(p3);
		btmesh.data(*f_it).addPoint(p3 * 0.5f + p1 * 0.5f);
	}

}

void partition(BezierTMesh &mesh, std::vector<BezierTMesh::VertexHandle> &sources)
{
	using VH = BezierTMesh::VertexHandle;
	using ID = unsigned int;

	// add a vertex property storing the computed region (id)
	auto id = OpenMesh::getOrMakeProperty<VH, ID>(mesh, "region");

	for (const auto &vertex : mesh.vertices()) {
		id[vertex] = 0;
	}

	ID i = 0;
	std::queue<VH> q;

	for (auto &vertex : sources) {
		id[vertex] = i++;
		q.push(vertex);
	}

	while (!q.empty()) {
		const auto _vh = q.front();
		q.pop();

		i = id[_vh];
		// give id to neighbors
		for (auto vv = mesh.vv_begin(_vh); vv != mesh.vv_end(_vh); ++vv) {
			if (id[*vv] == 0) {
				id[*vv] = i;
				q.push(*vv);
			}
		}
	}
}

void remesh(BezierTMesh &mesh)
{

}

void voronoi(BezierTMesh &mesh, unsigned int size)
{
	// source nodes
	std::vector<BezierTMesh::VertexHandle> sources;
	sources.reserve(size);

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);

	while (sources.size() < size) {
		for (auto &vh : mesh.vertices()) {
			if (sources.size() == size) {
				break;
			}
			if (dis(gen) > 0.5) {
				sources.push_back(vh);
			}
		}
	}

	partition(mesh, sources);
}

}
