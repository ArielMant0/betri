#include "BezierTriangleUtils.hh"

#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <queue>
#include <unordered_set>
#include <random>

#include <fstream>

namespace betri
{

template <class Container>
void partition(BezierTMesh &mesh, Container &sources)
{
	using VH = BezierTMesh::VertexHandle;
	using EH = BezierTMesh::EdgeHandle;
	using HH = BezierTMesh::HalfedgeHandle;
	using FH = BezierTMesh::FaceHandle;
	using P = BezierTMesh::Point;
	using ID = int;

	// queue to iterate over complete mesh
	std::queue<FH> q;
	// store computed regions
	std::vector<std::unordered_set<FH>> regions(sources.size());

	// temporary property to easily access region from vertex
	auto id = OpenMesh::makeTemporaryProperty<FH, ID>(mesh);

	for (auto &face : mesh.faces()) {
		id[face] = -1;
	}
	ID i = 0;
	for (auto &face : sources) {
		regions[i].insert(face);
		id[face] = i;
		q.push(face);
		i++;
	}

	while (!q.empty()) {
		auto f = q.front();
		q.pop();

		i = id[f];
		// give id to neighbors
		for (auto fh = mesh.ff_begin(f); fh != mesh.ff_end(f); ++fh) {
			if (id[*fh] == -1) {
				id[*fh] = i;
				regions[i].insert(*fh);
				q.push(*fh);
			}
			//} else if (id[*vv] != i) {
			//	// store neighbor relation
			//	auxData[i].second.insert(id[*vv]);
			//}
		}
	}

	/**
	 * Other Idea:
	 * - create regions (as before)
	 * - identify best corner vertices for each new face
	 * - create control vertices from other region vertices for each new face
	 */

	auto isInnerEdge = [&](const HH &he) {
		return !mesh.is_boundary(mesh.edge_handle(he)) &&
			id[mesh.face_handle(he)] == id[mesh.opposite_face_handle(he)];
	};

	auto addLinearFace = [&](VH vhs[3]) {
		const FH f = mesh.add_face(vhs, 3);
		mesh.addCPsToFace(f);
	};

	int lin = 0, comp = 0, inner = 0;

	std::ofstream out("voronoi-log.txt", std::ios::out);
	out << regions.size() << " regions\n\n";

	for (int idx = 0; idx < regions.size(); idx++) {
		auto set = regions[idx];

		out << "handling region " << idx << " with size: " << set.size() << std::endl;

		// if region is just a triangle dont do all the other stuff
		if (set.size() == 1) {
			mesh.addCPsToFace(*set.begin());
			lin++;
			continue;
		}

		FH face;
		// remove all edges from the faces that lie inside the region (merges adjacent faces)
		for (auto f = set.begin(); f != set.end(); ++f) {
			if (mesh.status(*f).deleted()) {
				out << "\tskipped a removed face" << std::endl;
				continue;
			}
			for (auto he = mesh.fh_begin(*f); he != mesh.fh_end(*f); ++he) {
				if (isInnerEdge(*he)) {
					inner++;
					auto &f2 = mesh.opposite_face_handle(*he);
					face = mesh.remove_edge(mesh.edge_handle(*he));
					out << "\tremoved a face" << std::endl;
					break;
				}
			}
		}
	}

	auto numVertices = [&](const FH &f) {
		int count = 0;
		for (auto v = mesh.cfv_begin(f); v != mesh.cfv_end(f); ++v, ++count) {}
		return count;
	};
	auto p2LineDistance = [&](const P &p1, const P &p2, const P &p) {
		return std::pow(((p - p2) % (p - p1)).norm(), 2.0) / (p2 - p1).norm();
	};

	mesh.garbage_collection();

	out << "\ncheck faces:" << std::endl;
	for (auto &face : mesh.faces()) {
		out << "\tface " << face << " has " << numVertices(face) << " vertices" << std::endl;
	}

	out << "\nfix face valences:" << std::endl;

	int fCount = 0;
	for (auto &face : mesh.faces()) {

		if (mesh.status(face).deleted()) {
			out << "skipping deleted face " << face << std::endl;
			continue;
		}
		out << "\ttouch up on face " << face << std::endl;

		fCount++;

		HH hTo;
		VH prev, next, toRemove;

		double dist;
		auto bt = mesh.data(face);
		int count = 0, verts = numVertices(face);

		bt.clear();
		out << "\t\tface " << face << " started out with " << verts << " vertices" << std::endl;

		while (verts > 3) {
			dist = std::numeric_limits<double>::max();
			prev = next = toRemove = BezierTMesh::InvalidVertexHandle;

			auto end = mesh.fh_end(face);
			for (auto h1 = mesh.fh_begin(face); h1 != end; ++h1) {
				// get next two vertices belonging to this face
				auto h2 = std::next(h1, 1);
				auto h3 = std::next(h2, 1);

				if (h2 == end) {
					h2 = mesh.fh_begin(face);
					h3 = std::next(h2, 1);
				} else if (h3 == end) {
					h3 = mesh.fh_begin(face);
				}

				auto &v1 = mesh.to_vertex_handle(h1);
				auto &v2 = mesh.to_vertex_handle(h2);
				auto &v3 = mesh.to_vertex_handle(h3);

				const auto val = mesh.valence(v2);
				out << "\t\t candidates: " << v1 << ", " << v2 << "(" << val <<"), " << v3 << std::endl;

				if (mesh.status(v1).deleted() || mesh.status(v2).deleted() ||
					mesh.status(v3).deleted()) continue;

				// check distance from v2 to the straight line through v1 to v3
				double d = p2LineDistance(mesh.point(v1), mesh.point(v3), mesh.point(v2));

				out << " with distance: " << d << std::endl;

				if (d < dist) {
					dist = d;
					prev = v1;
					next = v3;
					toRemove = v2;
					hTo = *h2;
				}
			}
			if (toRemove != BezierTMesh::InvalidVertexHandle &&
				prev != BezierTMesh::InvalidVertexHandle &&
				next != BezierTMesh::InvalidVertexHandle) {

				out << "\tremoving vertex " << toRemove << std::endl;
				bt.addPoint(mesh.point(toRemove));

				auto hKeep = hTo;
				auto hDel = mesh.next_halfedge_handle(hTo);

				auto hKeepOpp = mesh.opposite_halfedge_handle(hDel);
				auto hDelOpp = mesh.next_halfedge_handle(hKeepOpp);

				mesh.set_vertex_handle(hKeep, next);
				mesh.set_next_halfedge_handle(hKeep, mesh.next_halfedge_handle(hDel));
				mesh.set_vertex_handle(hKeepOpp, prev);
				mesh.set_next_halfedge_handle(hKeepOpp, mesh.next_halfedge_handle(hDelOpp));
				mesh.status(toRemove).set_deleted(true);
				mesh.status(hDel).set_deleted(true);
				mesh.status(hDelOpp).set_deleted(true);
				verts--;

				out << "\t" << verts << " vertices remain" << std::endl;
			}
		}
		out << "\tfinished face " << face << ", status: " << mesh.status(face).bits() << std::endl;
		comp++;
	}

	out << "\nnumber of faces is " << fCount << " and should be " << sources.size() << std::endl;

	out << lin << " linear faces\n";
	out << comp << " complex faces with " << inner << " inner edges\n";
	out.close();

	mesh.garbage_collection();

	// problem: center cannot be calculated, so where to start?

	// create mesh from regions (find shortest path over mesh between region barycenters)
	// for each region
	//		find triangle in which center lies
	//		while neighbor cog not reached
	//			find edge of current face that is closest to neighbor center
	//			add new edge that crosses closest edge in direction of neighbor center
}


void remesh(BezierTMesh &mesh)
{
}

void voronoi(BezierTMesh &mesh, unsigned int size)
{
	if (size > mesh.n_faces()) {
		return;
	} else if (size == mesh.n_faces()) {
		return partition(
			mesh, std::vector<BezierTMesh::FaceHandle>(mesh.faces_begin(), mesh.faces_end())
		);
	}
	// source nodes
	std::unordered_set<BezierTMesh::FaceHandle> sources;
	sources.reserve(size);

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);

	while (sources.size() < size) {
		for (auto &fh : mesh.faces()) {
			if (sources.size() == size) {
				break;
			}
			if (dis(gen) > 0.5) {
				sources.insert(fh);
			}
		}
	}

	partition(mesh, sources);
}

}
