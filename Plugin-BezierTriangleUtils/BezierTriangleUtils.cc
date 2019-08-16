#include "BezierTriangleUtils.hh"
#include "algorithms/Dijkstra.hh"

#include <queue>
#include <unordered_set>
#include <random>

#include <fstream>

namespace betri
{

/**
 * Performs face-based dijkstra on the mesh (for the given seeds)
 */
template <class Container>
void partition(BezierTMesh &mesh, Container &seeds)
{
	using VH = BezierTMesh::VertexHandle;
	using EH = BezierTMesh::EdgeHandle;
	using FH = BezierTMesh::FaceHandle;
	using ID = int;

#ifdef PRINT
	std::ofstream out("voronoi-log.txt", std::ios::out);
	out << seeds.size() << " seeds\n\n";
#endif

	const char *r = "region", *p = "predecessor", *d = "dist", *c = "crossed";
	dijkstra(mesh, seeds, r, p, d, c);

#ifdef PRINT
	out << "finished dijkstra" << std::endl;
#endif

	// property names: region, predecessor, distance, crossed
	auto id = OpenMesh::getProperty<FH, ID>(mesh, r);

	// new mesh
	BezierTMesh nmesh;

	// map regions IDs to new vertices
	std::vector<VH> vertices;
	vertices.reserve(seeds.size());

	// for each region
	for (const auto &face : seeds) {
		// add a vertex to the new mesh (store in vector to find using region id)
		vertices.push_back(nmesh.add_vertex(mesh.calc_face_centroid(face)));
	}

#ifdef PRINT
	out << "added vertices to new mesh" << std::endl;
#endif

	std::unordered_set<ID> faceIDs;
	std::vector<VH> corners;
	// for each vertex in the mesh
	for (const auto &v : mesh.vertices()) {
		// check how many different regions are around that vertex
		for (auto vf = mesh.vf_begin(v); vf != mesh.vf_end(v); ++vf) {
			if (faceIDs.insert(id[*vf]).second) {
				corners.push_back(vertices[id[*vf]]);
			}
		}

		if (faceIDs.size() > 1) {
			nmesh.add_face(corners);
		}
		faceIDs.clear();
		corners.clear();
	}

#ifdef PRINT
	out << "added faces to the new mesh" << std::endl;
#endif

	// does not work (also not with gc)
	mesh.clean_keep_reservation();
	//mesh.assign(nmesh);

	// does not work either (also not with gc)
	//mesh = nmesh;

	// works but seems really stupid
	for (const auto &v : nmesh.vertices()) {
		mesh.add_vertex_dirty(nmesh.point(v));
	}
	for (const auto &f : nmesh.faces()) {
		std::vector<VH> faces(nmesh.fv_begin(f), nmesh.fv_end(f));
#ifdef PRINT
		out << "adding face with " << faces.size() << " vertices\n";
#endif
		mesh.add_face(faces);
	}

	mesh.remove_property(*id);
	mesh.remove_property(*OpenMesh::getProperty<FH, FH>(mesh, p));
	mesh.remove_property(*OpenMesh::getProperty<FH, double>(mesh, d));
	mesh.remove_property(*OpenMesh::getProperty<EH, bool>(mesh, c));

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
		return;
	} else if (size == mesh.n_faces()) {
		return partition(
			mesh,
			std::vector<BezierTMesh::FaceHandle>(mesh.faces_begin(), mesh.faces_end())
		);
	}
	// source nodes
	std::unordered_set<BezierTMesh::FaceHandle> sources;
	sources.reserve(size);

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);

	auto neighbors = [&](const BezierTMesh::FaceHandle &f) {
		for (auto ff = mesh.cff_begin(f); ff != mesh.cff_end(f); ++ff) {
			if (sources.find(*ff) != sources.end()) {
				return true;
			}
		}
		return false;
	};

	while (sources.size() < size) {
		for (const auto &fh : mesh.faces()) {
			if (sources.size() == size) {
				break;
			}
			if (dis(gen) > 0.5) { // && !neighbors(fh)) {
				sources.insert(fh);
			}
		}
	}

	partition(mesh, sources);
}

}
