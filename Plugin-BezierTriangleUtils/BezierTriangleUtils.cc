#include "BezierTriangleUtils.hh"
#include "algorithms/Dijkstra.hh"

#include <queue>
#include <map>
#include <random>

namespace betri
{

/**
 * Performs face-based dijkstra on the mesh (for the given seeds)
 */
void partition(BezierTMesh &mesh, std::vector<FH> &seeds)
{
	//using VH = BezierTMesh::VertexHandle;
	//using EH = BezierTMesh::EdgeHandle;
	//using HH = BezierTMesh::HalfedgeHandle;
	//using FH = BezierTMesh::FaceHandle;
	//using ID = int;

#ifdef PRINT
	std::ofstream out("voronoi-log.txt", std::ios::out);
	out << seeds.size() << " seeds\n\n";
#endif

	dijkstra(mesh, seeds);

#ifdef PRINT
	out << "finished dijkstra" << std::endl;
#endif

	// property names: region, predecessor, distance, crossed
	auto id = OpenMesh::getProperty<FH, ID>(mesh, Dijkstra::REGION);
	auto pred = OpenMesh::getProperty<FH, FH>(mesh, Dijkstra::PREDECESSOR);
	auto dist = OpenMesh::getProperty<FH, double>(mesh, Dijkstra::DISTANCE);
	auto crossed = OpenMesh::getProperty<EH, bool>(mesh, Dijkstra::CROSSED);

	//std::vector<std::unordered_set<EH>> borders;
	//borders.reserve(seeds.size());
	//for (int i = 0; i < seeds.size(); ++i) {
	//	borders.push_back(std::unordered_set<EH>());
	//}
	//for (auto &edge : mesh.edges()) {
	//	if (!crossed[edge]) {
	//		borders[id[mesh.face_handle(mesh.halfedge_handle(edge, 0))]].insert(edge);
	//		borders[id[mesh.face_handle(mesh.halfedge_handle(edge, 1))]].insert(edge);
	//	}
	//}
	// new mesh
	BezierTMesh nmesh;

	// map regions IDs to new vertices
	std::vector<VH> vertices;
	vertices.reserve(seeds.size());

	// for each region: add a vertex to the new mesh (store in vector to find using region id)
	std::transform(seeds.begin(), seeds.end(), vertices.begin(), [&](FH &f) {
		return nmesh.add_vertex(mesh.calc_face_centroid(f));
	});

#ifdef PRINT
	out << "added vertices to new mesh" << std::endl;
#endif

	// find next edge of a region border
	// (always return the halfedge that is adjacent to face with id f1)
	auto findNextBorder = [&](const VH &v, ID f1, EH forbidden) {
		for (auto h = mesh.voh_begin(v); h != mesh.voh_end(v); ++h) {
			const ID id1 = id[mesh.face_handle(*h)];
			const ID id2 = id[mesh.opposite_face_handle(*h)];
			const EH edge = mesh.edge_handle(*h);
			// if edge was not crossed and is adjacent to the given region
			if (!crossed[edge] && edge != forbidden && id1 == f1 && id2 != f1) {
				return *h;
			} else if (!crossed[edge] && edge != forbidden && id1 != f1 && id2 != f1) {
				return mesh.opposite_halfedge_handle(*h);
			}
		}
		return INVALID_H;
	};

	auto findCrossedEdge = [&](FH &f, EH &forbidden, ID region) {
#ifdef PRINT
		out << "\tlooking for crossed edge not " << forbidden << std::endl;
#endif
		for (auto fh = mesh.fh_begin(f); fh != mesh.fh_end(f); ++fh) {
			const HH he = mesh.opposite_halfedge_handle(fh);
			const EH e = mesh.edge_handle(he);
#ifdef PRINT
			out << "\t\t\tedge (" << e << ") was" << (crossed[e] ? "" : " NOT");
			out << " crossed (opp face region = " << id[mesh.face_handle(he)];
			out << ", face region = " << id[mesh.opposite_face_handle(he)] << ")\n";
#endif
			if (crossed[e] && e != forbidden && id[mesh.face_handle(he)] == region) {
				return e;
			}
		}
		return INVALID_E;
	};

	auto hasVertex = OpenMesh::makeTemporaryProperty<EH, VH>(mesh);
	for (auto &edge : mesh.edges()) {
		hasVertex[edge] = INVALID_V;
	}

	//auto firstBorder = [&](EH edge, ID faceId) {
	//	HH h1 = mesh.halfedge_handle(edge, 0), h2 = mesh.halfedge_handle(edge, 1);
	//	return id[mesh.face_handle(h1)] == faceId ? h1 : h2;
	//};

	auto shortestPath = [&](const VH &v, const FH &f1, const FH &f2) {
		const ID id_1 = id[f1], id_2 = id[f2];

#ifdef PRINT
		out << "trying to find path between regions " << id_1 << " and " << id_2 << "\n";
#endif
		HH start = findNextBorder(v, id_1, INVALID_E);
		//auto b_it = borders[id_1].begin();
		//HH start = firstBorder(*b_it, id_1);
		HH he = start, begin = start;

		FH f11, f22;
		FH tmp1 = mesh.face_handle(he), tmp2 = mesh.opposite_face_handle(he);
		double sum = std::numeric_limits<double>::max();

		// find faces with shortest path to their respective seed points
		do {
			const double distSum = dist[tmp1] + dist[tmp2];
			if (distSum < sum && id[mesh.opposite_face_handle(he)] == id_2) {
#ifdef PRINT
				out << "\t\tnew distance " << distSum << " < " << sum << "\n";
#endif
				sum = distSum;
				f11 = tmp1;
				f22 = tmp2;
				start = he;
			}

			//b_it++;
			//if (b_it == borders[id_1].end()) b_it = borders[id_1].begin();

			he = findNextBorder(mesh.to_vertex_handle(he), id_1, mesh.edge_handle(he));
			tmp1 = mesh.face_handle(he);
			tmp2 = mesh.opposite_face_handle(he);
		} while (he != INVALID_H && he != begin && id[tmp1] == id_1 && id[tmp2] == id_2);

#ifdef PRINT
		if (sum == std::numeric_limits<double>::max() || start == INVALID_H) {
			out << "\tcould not find faces or starting edge !!! " << start << "\n";
		} else {
			out << "\tfound faces with shortest distance (" << sum << ") at border\n";
		}
#endif

		EH e1 = mesh.edge_handle(start), e2 = mesh.edge_handle(start), edge;

		std::deque<VH> path;
		if (hasVertex[e1] == INVALID_V) {
			hasVertex[e1] = nmesh.add_vertex(mesh.calc_edge_midpoint(e1));
		}
		path.push_back(hasVertex[e1]);

		while (pred[f11] != INVALID_F) {
			// find next edge
			edge = findCrossedEdge(f11, e1, id_1);
			if (edge == INVALID_E) {
#ifdef PRINT
				out << "\tERROR 1: could not find next crossed edge";
				out << " (predecessor: " << pred[f11] << ", target: " << f1;
				out << ", region: " << id[f11] << ')' << std::endl;
#endif
				break;
			}
			if (hasVertex[edge] == INVALID_V) {
				hasVertex[edge] = nmesh.add_vertex(mesh.calc_edge_midpoint(edge));
			}
			path.push_front(hasVertex[edge]);
			e1 = edge;
			f11 = pred[f11];
		}

#ifdef PRINT
		out << "\tadded path vertices for region 1 (current path length: ";
		out << path.size() << ')' << std::endl;
#endif
		while (pred[f22] != INVALID_F) {
			edge = findCrossedEdge(f22, e2, id_2);
			if (edge == INVALID_E) {
#ifdef PRINT
				out << "\tERROR 2: could not find next crossed edge";
				out << " (predecessor: " << pred[f22] << ", target: " << f2;
				out << ", region: " << id[f22] << ')' << std::endl;
#endif
				break;
			}
			if (hasVertex[edge] == INVALID_V) {
				hasVertex[edge] = nmesh.add_vertex(mesh.calc_edge_midpoint(edge));
			}
			path.push_back(hasVertex[edge]);
			e2 = edge;
			f22 = pred[f22];
		}
#ifdef PRINT
		out << "\tadded path vertices for region 2 (total path length: ";
		out << path.size() << ')' << std::endl;
#endif
		return path;
	};

	std::unordered_set<FH> seedFaces;
	std::vector<VH> points;
	// for each vertex in the mesh
	for (const auto &v : mesh.vertices()) {
		// check how many different regions are around that vertex
		for (auto vf = mesh.vf_begin(v); vf != mesh.vf_end(v); ++vf) {
			seedFaces.insert(seeds[id[*vf]]);
#ifdef PRINT
			out << "vertex adj to region " << id[*vf] << "\n";
			int i = 0;
			for (auto eh = mesh.fe_begin(*vf); eh != mesh.fe_end(*vf); ++eh) {
				if (!crossed[*eh]) i++;
			}
			out << "\tface is incident to " << i << " borders\n";
#endif
		}

#ifdef PRINT
			out << "vertex adj to " << seedFaces.size() << " regions" << std::endl;
#endif
		// only do sth if we found at least 3 regions
		if (seedFaces.size() > 2) {
			for (auto f = seedFaces.begin(); f != seedFaces.end(); ++f) {
				points.push_back(vertices[id[*f]]);
				// get next face
				auto next = std::next(f, 1);
				if (next == seedFaces.end()) {
					next = seedFaces.begin();
				}
				// add all points from shortest path between two regions
				auto path = shortestPath(v, *f, *next);
				points.insert(points.end(), path.begin(), path.end());
			}
			nmesh.add_face(points);
			points.clear();
		}

//			for (auto vhe = mesh.voh_begin(v); vhe != mesh.voh_end(v); ++vhe) {
//				points.push_back(vertices[id[*f]]);
//				// get next face
//				auto next = std::next(f, 1);
//				if (next == faceIDs.end()) {
//					next = faceIDs.begin();
//				}
//				// add all points from shortest path
//				auto path = shortestPath(v, *f, *next);
//				points.insert(points.end(), path.begin(), path.end());
//#ifdef PRINT
//				out << "found shortest path between regions " << id[*f] << " and ";
//				out << id[*next] << " with a length of " << points.size() + 1 << std::endl;
//#endif
//			}
//			nmesh.add_face(points);
//			points.clear();
//		}
		seedFaces.clear();
	}

#ifdef PRINT
	out << "added faces to the new mesh" << std::endl;
#endif

	mesh.clean_keep_reservation();
	// does not work (also not with gc)
	//mesh.assign(nmesh);

	// does not work either (also not with gc)
	//mesh = nmesh;

	// works but seems really stupid
	for (const auto &v : nmesh.vertices()) {
		mesh.add_vertex_dirty(nmesh.point(v));
	}
	for (const auto &f : nmesh.faces()) {
		std::vector<VH> faces(nmesh.fv_begin(f), nmesh.fv_end(f));
		mesh.add_face(faces);
	}

	mesh.remove_property(*id);
	mesh.remove_property(*pred);
	mesh.remove_property(*dist);
	mesh.remove_property(*crossed);

	mesh.garbage_collection();

#ifdef PRINT
	out.close();
#endif
}


void remesh(BezierTMesh &mesh)
{
}

void voronoi(BezierTMesh &mesh, unsigned int size)
{
	if (size > mesh.n_faces()) {
		size = mesh.n_faces();
	}

	// source nodes
	std::vector<FH> seeds;
	seeds.reserve(size);

	// mostly used for debugging
	if (size == mesh.n_faces()) {
		return partition(mesh, std::vector<FH>(mesh.faces_begin(), mesh.faces_end()));
	}

	std::unordered_set<FH> check;
	check.reserve(size);

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);

	//auto neighbors = [&](const BezierTMesh::FaceHandle &f) {
	//	for (auto ff = mesh.cff_begin(f); ff != mesh.cff_end(f); ++ff) {
	//		if (seeds.find(*ff) != seeds.end()) {
	//			return true;
	//		}
	//	}
	//	return false;
	//};

	while (seeds.size() < size) {
		for (const auto &fh : mesh.faces()) {
			if (seeds.size() == size) {
				break;
			}
			if (dis(gen) > 0.5 && check.insert(fh).second) { // && !neighbors(fh)) {
				seeds.push_back(fh);
			}
		}
	}

	partition(mesh, seeds);
}

}
