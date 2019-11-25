//	Copyright (c) 2013, Andre Gaschler, Quirin Fischer
//	All rights reserved.
//
//	Redistribution and use in source and binary forms, with or without
// modification,
//	are permitted provided that the following conditions are met:
//
//	* Redistributions of source code must retain the above copyright notice,
// this
//	  list of conditions and the following disclaimer.
//
//	* Redistributions in binary form must reproduce the above copyright
// notice, this
//	  list of conditions and the following disclaimer in the documentation
// and/or
//	  other materials provided with the distribution.
//
//	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND
//	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED
//	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR
//	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES
//	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES;
//	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON
//	ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS
//	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "MetricGenerator.hh"

namespace boundingmesh
{
MetricGenerator::MetricGenerator()
{}

MetricGenerator::~MetricGenerator()
{}

void MetricGenerator::setMesh(::std::shared_ptr<TriMesh> mesh)
{
	mesh_ = mesh;
	initialize();
}

void MetricGenerator::setMetric(Metric metric)
{
	metric_ = metric;
	initialize();
}

void MetricGenerator::setInitialization(Initialization initialization)
{
	initialization_ = initialization;
	initialize();
}

Matrix44 MetricGenerator::getErrorMetric(TriMesh::EdgeHandle edge_index)
{
	Matrix44 qem = Matrix44::Zero();
	TriMesh::HalfedgeHandle he = mesh_->halfedge_handle(edge_index, 0);
	auto v0 = mesh_->to_vertex_handle(he);
	auto v1 = mesh_->from_vertex_handle(he);

	switch (metric_) {
		case ClassicQEM:
			qem = qems_[v0.idx()] + qems_[v1.idx()];
			break;
		case ModifiedQEM:
			qem = vertices_qem_[v0.idx()] + vertices_qem_[v1.idx()];
			// qem -= edges_qem_[edge_index];
			break;
		case MinimizedConstant:
		case Diagonalization:
		case Average:
			qem = mergeMatrices(qems_merge_[v0.idx()],
				qems_merge_[v1.idx()]);
			break;
		default:
			break;
	}

	// Debugging: QEM matrices should be positive-semidefinite
	Eigen::SelfAdjointEigenSolver<Matrix44> es;
	es.compute(qem);
	if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
		es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
		std::cout << "Edge " << edge_index << std::endl;
		std::cout << "Bad metric eigenvalues: " << es.eigenvalues().transpose()
			<< std::endl;
		// std::cout << edge.vertex(0) << ": " << std::endl <<
		// vertices_qem_[edge.vertex(0)] << std::endl << edge.vertex(1) << ": " <<
		// vertices_qem_[edge.vertex(1)] << std::endl << "edge " << edge_index << ":
		// " << edges_qem_[edge_index] << std::endl;
	}
	return qem;
}

void MetricGenerator::contractEdge(TriMesh::EdgeHandle edge_index)
{
	TriMesh::HalfedgeHandle he = mesh_->halfedge_handle(edge_index, 0);
	auto v0 = mesh_->to_vertex_handle(he);
	auto v1 = mesh_->from_vertex_handle(he);

	switch (metric_) {
		case ClassicQEM:
			// The contraction removes two vertices from the mesh and inserts the new
			// one.
			// Since the freed indices are managed in a stack, the last one to be
			// deleted will be the new index.
			// The removal is performed in ascending order -> first remove(v0), then
			// remove(v1)
			qems_[v1.idx()] = getErrorMetric(edge_index);
			break;
		case ModifiedQEM:
			contractEdgeModifiedQEM(edge_index);
			break;
		case MinimizedConstant:
		case Diagonalization:
		case Average:
			// See ClassicQEM for indexing method
			qems_merge_[v1.idx()] = getErrorMetric(edge_index);
			break;
		default:
			break;
	}
}

void MetricGenerator::contractEdgeModifiedQEM(TriMesh::EdgeHandle edge_index)
{
	/*
			This method depends on the implementation of
	   Decimator::executeEdgeContraction
			and the Mesh add/remove methods. It essentially simulates the mesh
	   changes
			to get the correct indices for new vertices, edges and triangles.
	*/
	TriMesh::HalfedgeHandle he = mesh_->halfedge_handle(edge_index, 0);
	auto v0 = mesh_->to_vertex_handle(he);
	auto v1 = mesh_->from_vertex_handle(he);

	Matrix44 new_vertex_qem = getErrorMetric(edge_index);

	// An explanation of these data structures can be found in the method that
	// uses them, collectRemovalData.
	std::vector<HoleEdge, Eigen::aligned_allocator<HoleEdge> > hole_border;
	std::map<Index, Matrix44, std::less<Index>,
		Eigen::aligned_allocator<std::pair<const Index, Matrix44> > >
		new_edges_qem;

	collectRemovalData(v0, v1, hole_border,
		new_edges_qem);
	collectRemovalData(v1, v0, hole_border,
		new_edges_qem);

	for (unsigned int i = 0; i < hole_border.size(); ++i) {
		Eigen::SelfAdjointEigenSolver<Matrix44> es;
		es.compute(hole_border[i].old_triangle_qem);
		if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
			es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
			std::cout << "Hole border " << i << std::endl;
			std::cout << "Bad metric eigenvalues: " << es.eigenvalues().transpose()
				<< std::endl;
			// std::cout << edge.vertex(0) << ": " << std::endl <<
			// vertices_qem_[edge.vertex(0)] << std::endl << edge.vertex(1) << ": " <<
			// vertices_qem_[edge.vertex(1)] << std::endl << "edge " << edge_index <<
			// ": " << edges_qem_[edge_index] << std::endl;
		}
	}

	for (std::map<Index, Matrix44, std::less<Index>,
		Eigen::aligned_allocator<std::pair<const Index, Matrix44> > >::
		iterator it = new_edges_qem.begin();
		it != new_edges_qem.end(); ++it) 
	{
		Eigen::SelfAdjointEigenSolver<Matrix44> es;
		es.compute(it->second);
		if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
			es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
			std::cout << "Hole vertex " << it->first << std::endl;
			std::cout << "Bad metric eigenvalues: " << es.eigenvalues().transpose()
				<< std::endl;
			// std::cout << edge.vertex(0) << ": " << std::endl <<
			// vertices_qem_[edge.vertex(0)] << std::endl << edge.vertex(1) << ": " <<
			// vertices_qem_[edge.vertex(1)] << std::endl << "edge " << edge_index <<
			// ": " << edges_qem_[edge_index] << std::endl;
		}
	}

	std::stack<TriMesh::FaceHandle> deleted_triangles;
	std::set<TriMesh::FaceHandle> triangles_already_deleted;
	std::stack<TriMesh::EdgeHandle> deleted_edges;
	std::map<TriMesh::EdgeHandle, unsigned int> edges_with_deleted_tris;
	std::set<TriMesh::VertexHandle> vertices_of_inserted_border_edges;

	// Removal of triangles and their edges
	for (TriMesh::VertexFaceIter vf_it = mesh_->vf_iter(v0); vf_it.is_valid(); ++vf_it) {

		TriMesh::FaceHandle triangle_index = *vf_it;

		if (triangles_already_deleted.find(triangle_index) == triangles_already_deleted.end()) {

			deleted_triangles.push(triangle_index);
			triangles_already_deleted.insert(triangle_index);

			for (TriMesh::FaceEdgeIter fe_it = mesh_->fe_iter(*vf_it); fe_it.is_valid(); ++fe_it) {
				if (mesh_->is_boundary(*fe_it)) {
					deleted_edges.push(*fe_it);
				} else {
					if (edges_with_deleted_tris.find(*fe_it) != edges_with_deleted_tris.end() && edges_with_deleted_tris[*fe_it] == 1)
						deleted_edges.push(*fe_it);
					else if (edges_with_deleted_tris.find(*fe_it) != edges_with_deleted_tris.end())
						edges_with_deleted_tris[*fe_it] = edges_with_deleted_tris[*fe_it] - 1;
					else
						edges_with_deleted_tris[*fe_it] = 1;
				}
			}
		}
	}
	for (TriMesh::VertexFaceIter vf_it = mesh_->vf_iter(v1); vf_it.is_valid(); ++vf_it) {

		TriMesh::FaceHandle triangle_index = *vf_it;

		if (triangles_already_deleted.find(triangle_index) == triangles_already_deleted.end()) {

			deleted_triangles.push(triangle_index);
			triangles_already_deleted.insert(triangle_index);

			for (TriMesh::FaceEdgeIter fe_it = mesh_->fe_iter(*vf_it); fe_it.is_valid(); ++fe_it) {
				if (mesh_->is_boundary(*fe_it)) {
					deleted_edges.push(*fe_it);
				} else {
					if (edges_with_deleted_tris.find(*fe_it) !=
						edges_with_deleted_tris.end() &&
						edges_with_deleted_tris[*fe_it] == 1)
						deleted_edges.push(*fe_it);
					else if (edges_with_deleted_tris.find(*fe_it) !=
						edges_with_deleted_tris.end())
						edges_with_deleted_tris[*fe_it] =
						edges_with_deleted_tris[*fe_it] - 1;
					else
						edges_with_deleted_tris[*fe_it] = 1;
				}
			}
		}
	}

	TriMesh::VertexHandle new_vertex_index = v1;
	vertices_qem_[new_vertex_index.idx()] = new_vertex_qem;

	// Updating all edge/triangle QEMs
	for (unsigned int i = 0; i < hole_border.size(); ++i) {
		TriMesh::FaceHandle new_triangle_index = deleted_triangles.top();
		deleted_triangles.pop();
		triangles_qem_[new_triangle_index.idx()] = hole_border[i].old_triangle_qem;

		if (vertices_of_inserted_border_edges.find(hole_border[i].vertex_1) == vertices_of_inserted_border_edges.end()) {
			assert(deleted_edges.size() > 0);

			TriMesh::EdgeHandle new_edge_index = deleted_edges.top();
			deleted_edges.pop();

			vertices_of_inserted_border_edges.insert(hole_border[i].vertex_1);
			edges_qem_[new_edge_index.idx()] = new_edges_qem[hole_border[i].vertex_1.idx()];
		}
		if (vertices_of_inserted_border_edges.find(hole_border[i].vertex_2) == vertices_of_inserted_border_edges.end()) {
			assert(deleted_edges.size() > 0);

			TriMesh::EdgeHandle new_edge_index = deleted_edges.top();
			deleted_edges.pop();

			vertices_of_inserted_border_edges.insert(hole_border[i].vertex_2);
			edges_qem_[new_edge_index.idx()] = new_edges_qem[hole_border[i].vertex_2.idx()];
		}
	}
}

void MetricGenerator::collectRemovalData(
	TriMesh::VertexHandle vertex_index, TriMesh::VertexHandle other_index,
	std::vector<HoleEdge, Eigen::aligned_allocator<HoleEdge> >& hole_border,
	std::map<Index, Matrix44, std::less<Index>,
	Eigen::aligned_allocator<std::pair<const Index, Matrix44> > >&
	new_edges_qem)
{
	/*
			Hole border:
					Edges that will become the mesh border when the vertex is
	   deleted.
					For every neighbouring triangle of the removed vertex,
					this is the one edge not containing the vertex.
					In the following picture, the edges 1, 2, 3 are border edges.
							3
						 ___v___
						/\     /\
					1->/  \   /  \<-2
					  /____\./____\
							^
					  The vertex to be removed

			Shared triangles:
					Triangles that contain both vertices of the removed edge.
					These do not produce a border edge (since 2 vertices get
	   removed).
					Also, the QEM matrix for the new edge that will be inserted is
	   computed
					differently than for non-shared triangles.
					Example: The edge that will be removed connects vertices 1 and
	   2.
							The two adjacent triangles are shared (marked with
	   "sh").
						 ______________
						/\     /\     /\
					   /  \   /sh\   /  \
					  /____\1/____\2/____\
					  \    / \    / \    /
					   \  /   \sh/   \  /
						\/_____\/_____\/

			Stored matrices:
					Every border edge stores the QEM matrix of the triangle that
	   generated it.
					This allows us to restore the QEM for the new triangle that
	   will be formed
					with the border edge and the new vertex. Also, for every point
	   on the
					hole border the QEM of the edge that connected it to the
	   removed
					vertex is stored(this is the map new_edges_qem). This data is
	   needed for
					the new edge that will connect it to the inserted vertex.
					For the third vertex of shared triangles, both the edges to
	   removed
					vertices as well as the shared triangle are incorporated to
	   compute the QEM.

					Summarizing, the matrices on the hole border edges are used to
	   generate new triangles, while
					the matrices in the map new_edges_qem are used for new edges
	   (but the map identifies them
					by the vertex that will define the edge).


	*/
	for (TriMesh::VertexFaceIter vf_it = mesh_->vf_iter(vertex_index); vf_it.is_valid(); ++vf_it) {

		HoleEdge border_edge;
		bool is_shared_triangle = false;

		auto fv = mesh_->fv_iter(*vf_it);
		auto v0 = *fv; fv++;
		auto v1 = *fv; fv++;
		auto v2 = *fv;

		for (TriMesh::FaceVertexIter fv_it = mesh_->fv_iter(*vf_it); fv_it.is_valid(); ++fv_it) {
			if (*fv_it == other_index)
				is_shared_triangle = true;
		}

		if (!is_shared_triangle) {
			if (v0 == vertex_index) {
				border_edge.vertex_1 = v1;
				border_edge.vertex_2 = v2;
			} else if (v1 == vertex_index) {
				border_edge.vertex_1 = v2;
				border_edge.vertex_2 = v0;
			} else if (v2 == vertex_index) {
				border_edge.vertex_1 = v0;
				border_edge.vertex_2 = v1;
			}
			border_edge.old_triangle_qem = triangles_qem_[(*vf_it).idx()];

			hole_border.push_back(border_edge);
			// Save qem for edges to be created
			for (TriMesh::FaceEdgeIter fe_it = mesh_->fe_iter(*vf_it); fe_it.is_valid(); ++fe_it) {
				TriMesh::VertexHandle hole_vertex = TriMesh::VertexHandle(0);

				TriMesh::HalfedgeHandle he = mesh_->halfedge_handle(*fe_it, 0);
				auto v0 = mesh_->to_vertex_handle(he);
				auto v1 = mesh_->from_vertex_handle(he);

				if (v0 == vertex_index)
					hole_vertex = v1;
				else if (v1 == vertex_index)
					hole_vertex = v0;
				else
					continue;

				TriMesh::EdgeHandle edge_index = *fe_it;
				if (new_edges_qem.count(hole_vertex.idx()) == 0)
					new_edges_qem[hole_vertex.idx()] = edges_qem_[edge_index.idx()];
				else
					new_edges_qem[hole_vertex.idx()] += edges_qem_[edge_index.idx()];
			}
		} else {
			TriMesh::VertexHandle hole_vertex;
			for (TriMesh::FaceVertexIter fv_it = mesh_->fv_iter(*vf_it); fv_it.is_valid(); ++fv_it) {
				if (*fv_it != vertex_index && *fv_it != other_index) {
					hole_vertex = *fv_it;
					break;
				}
			}
			if (new_edges_qem.count(hole_vertex.idx()) == 0)
				new_edges_qem[hole_vertex.idx()] = -triangles_qem_[(*vf_it).idx()];
			else
				new_edges_qem[hole_vertex.idx()] -= triangles_qem_[(*vf_it).idx()];
		}
	}
}

void MetricGenerator::initialize()
{
	if (mesh_ == NULL) return;

	switch (metric_) {
		case ClassicQEM:
			qems_.clear();
			for (TriMesh::VertexIter v_it = mesh_->vertices_sbegin(); v_it != mesh_->vertices_end(); ++v_it)
				qems_.push_back(computeQEM(*v_it));
			break;
		case ModifiedQEM:
			vertices_qem_.clear();
			for (TriMesh::VertexIter v_it = mesh_->vertices_sbegin(); v_it != mesh_->vertices_end(); ++v_it)
				vertices_qem_.push_back(computeModifiedVertexQEM(*v_it));

			edges_qem_.clear();
			for (TriMesh::EdgeIter e_it = mesh_->edges_sbegin(); e_it != mesh_->edges_end(); ++e_it)
				edges_qem_.push_back(computeModifiedEdgeQEM(*e_it));

			triangles_qem_.clear();
			for (TriMesh::FaceIter f_it = mesh_->faces_sbegin(); f_it != mesh_->faces_end(); ++f_it) {
				auto vh = mesh_->fv_iter(*f_it);
				auto tmp = mesh_->point(vh++);
				auto v0 = Vector3(tmp[0], tmp[1], tmp[2]);
				tmp = mesh_->point(vh++);
				auto v1 = Vector3(tmp[0], tmp[1], tmp[2]);
				tmp = mesh_->point(vh);
				auto v2 = Vector3(tmp[0], tmp[1], tmp[2]);
				Plane p = Plane(v0, v1, v2);
				triangles_qem_.push_back(p.distanceMatrix());
			}
			break;
		case MinimizedConstant:
		case Diagonalization:
		case Average:
			qems_merge_.clear();
			for (TriMesh::VertexIter v_it = mesh_->vertices_sbegin(); v_it != mesh_->vertices_end(); ++v_it)
				qems_merge_.push_back(computeInitialMergeMetric(*v_it));
			break;
		default:
			break;
	}
}

Matrix44 MetricGenerator::computeQEM(TriMesh::VertexHandle vertex_index)
{
	Matrix44 qem = Matrix44::Zero();
	for (TriMesh::VertexFaceIter vf_it = mesh_->vf_iter(vertex_index); vf_it->is_valid(); ++vf_it) {
		auto vh = mesh_->fv_iter(*vf_it);
		auto tmp = mesh_->point(vh++);
		auto v0 = Vector3(tmp[0], tmp[1], tmp[2]);
		tmp = mesh_->point(vh++);
		auto v1 = Vector3(tmp[0], tmp[1], tmp[2]);
		tmp = mesh_->point(vh);
		auto v2 = Vector3(tmp[0], tmp[1], tmp[2]);
		Plane p = Plane(v0, v1, v2);
		triangles_qem_.push_back(p.distanceMatrix());
	}
	return qem;
}

Matrix44 MetricGenerator::computeModifiedVertexQEM(TriMesh::VertexHandle vertex_index)
{
	Matrix44 qem = Matrix44::Zero();
	std::vector<Vector3> normals;
	// Distance to triangles
	for (TriMesh::VertexFaceIter vf_it = mesh_->vf_iter(vertex_index); vf_it->is_valid(); ++vf_it) {
		auto vh = mesh_->fv_iter(*vf_it);
		auto tmp = mesh_->point(vh++);
		auto v0 = Vector3(tmp[0], tmp[1], tmp[2]);
		tmp = mesh_->point(vh++);
		auto v1 = Vector3(tmp[0], tmp[1], tmp[2]);
		tmp = mesh_->point(vh);
		auto v2 = Vector3(tmp[0], tmp[1], tmp[2]);
		Plane p = Plane(v0, v1, v2);
		normals.push_back(p.normal);
		triangles_qem_.push_back(p.distanceMatrix());
	}

	// Correction of distances to edges
	for (TriMesh::VertexEdgeIter ve_it = mesh_->ve_iter(vertex_index); ve_it->is_valid(); ++ve_it) {
		TriMesh::HalfedgeHandle edge = mesh_->halfedge_handle(*ve_it, 0);
		auto t0 = mesh_->face_handle(edge);
		auto t1 = mesh_->opposite_face_handle(edge);
		auto v0 = mesh_->to_vertex_handle(edge);
		auto v1 = mesh_->from_vertex_handle(edge);

		auto vh = mesh_->fv_iter(t0);
		auto tmp = mesh_->point(vh++);
		auto v00 = Vector3(tmp[0], tmp[1], tmp[2]);
		tmp = mesh_->point(vh++);
		auto v10 = Vector3(tmp[0], tmp[1], tmp[2]);
		tmp = mesh_->point(vh);
		auto v20 = Vector3(tmp[0], tmp[1], tmp[2]);
		Plane p1 = Plane(v00, v10, v20);

		vh = mesh_->fv_iter(t1);
		tmp = mesh_->point(vh++);
		auto v01 = Vector3(tmp[0], tmp[1], tmp[2]);
		tmp = mesh_->point(vh++);
		auto v11 = Vector3(tmp[0], tmp[1], tmp[2]);
		tmp = mesh_->point(vh);
		auto v21 = Vector3(tmp[0], tmp[1], tmp[2]);
		Plane p2 = Plane(v01, v11, v21);

		Vector3 normal1 = p1.normal;
		Vector3 normal2 = p2.normal;

		if (mesh_->is_boundary(*ve_it)) {
			tmp = mesh_->point(v0);
			Vector3 third_point = Vector3(tmp[0], tmp[1], tmp[2]) + normal1;
			tmp = mesh_->point(v0);
			auto tmp2 = mesh_->point(v1);
			Plane border_plane = Plane(Vector3(tmp[0], tmp[1], tmp[2]), Vector3(tmp2[0], tmp2[1], tmp2[2]), third_point);
			normals.push_back(border_plane.normal);
			qem += border_plane.distanceMatrix();
		} else {
			Real angle = std::acos(normal1.dot(normal2));  // in radians
			Real alpha = std::cos(3.141592653589793238462 - angle);
			if (alpha > epsilon) {
				Vector3 normal_interpolated = (normal1 + normal2).normalized();
				tmp = mesh_->point(v0);
				Real d = normal_interpolated.dot(Vector3(tmp[0], tmp[1], tmp[2]));
				Plane new_plane(normal_interpolated, d);
				normals.push_back(normal_interpolated);
				qem += alpha * new_plane.distanceMatrix();
			}
		}
	}

	// Correction of distance to vertex
	unsigned int n_normals = normals.size();
	Vector3 n_star = Vector3::Zero();
	Eigen::Matrix<Real, 3, 3> sum_Ni = Eigen::Matrix<Real, 3, 3>::Zero();
	Eigen::Matrix<Real, 3, Eigen::Dynamic> T;
	T.resize(3, n_normals);
	for (unsigned int i = 0; i < n_normals; ++i) {
		n_star += normals[i];
		sum_Ni += normals[i] * normals[i].transpose();
		T.col(i) << normals[i];
	}

	n_star.normalize();
	Eigen::Matrix<Real, 3, 3> N_star = n_star * n_star.transpose();

	Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> M_1;
	M_1.resize(n_normals, n_normals);
	M_1 = T.transpose() * T;
	Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> M_2;
	M_2.resize(n_normals, n_normals);
	M_2 = T.transpose() * sum_Ni * T;
	Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> M_3;
	M_3.resize(n_normals, n_normals);
	M_3 = T.transpose() * N_star * T;
	bool possible = true;
	Real lambda_min = -std::numeric_limits<Real>::max();
	Real lambda_max = std::numeric_limits<Real>::max();
	for (unsigned int i = 0; i < n_normals; ++i) {
		for (unsigned int j = 0; j < n_normals; ++j) {
			if (M_3(i, j) == 0) {
				if (M_1(i, j) - M_2(i, j) < 0) {
					// do nothing, is always ok
				} else {
					// std::cout<<"Impossible to correct "<<vertex_index<<":
					// "<<i<<","<<j<<std::endl;
					possible = false;
					break;
				}
			} else if (M_3(i, j) > 0) {
				Real new_lower_bound = (M_1(i, j) - M_2(i, j)) / M_3(i, j);
				if (new_lower_bound > lambda_min) {
					lambda_min = new_lower_bound;
				}
			} else if (M_3(i, j) < 0) {
				Real new_upper_bound = (M_1(i, j) - M_2(i, j)) / M_3(i, j);
				if (new_upper_bound < lambda_max) {
					lambda_max = new_upper_bound;
				}
			}
		}
		if (!possible) break;
	}

	if (lambda_min > lambda_max) {
		possible = false;
		// std::cout<<"bad interval: min "<<lambda_min<<" max
		// "<<lambda_max<<std::endl;
	}
	if (!possible) {
		// std::cout<<"Impossible to get valid lamdba"<<std::endl;
	}

	auto tmp = mesh_->point(vertex_index);
	Plane correction_plane(n_star, Vector3(tmp[0], tmp[1], tmp[2]).dot(n_star));
	if (lambda_min < 0) lambda_min = 0;

	qem += lambda_min * correction_plane.distanceMatrix();

	// Hack: add distance to vertex
	Matrix44 distance;
	distance << Matrix44::Identity();
	distance.col(3) << -Vector3(tmp[0], tmp[1], tmp[2]), 0;
	distance = distance.transpose() * distance;
	qem += distance_factor_ * distance;

	return qem;
}

Matrix44 MetricGenerator::computeModifiedEdgeQEM(TriMesh::EdgeHandle edge_index)
{
	Matrix44 qem = Matrix44::Zero();
	TriMesh::HalfedgeHandle he = mesh_->halfedge_handle(edge_index, 0);
	auto t0 = mesh_->face_handle(he);
	auto t1 = mesh_->opposite_face_handle(he);
	auto v0 = mesh_->to_vertex_handle(he);
	auto v1 = mesh_->from_vertex_handle(he);

	auto vh = mesh_->fv_iter(t0);
	auto tmp = mesh_->point(vh++);
	Vector3 v00 = Vector3(tmp[0], tmp[1], tmp[2]);
	tmp = mesh_->point(vh++);
	Vector3 v10 = Vector3(tmp[0], tmp[1], tmp[2]);
	tmp = mesh_->point(vh);
	Vector3 v20 = Vector3(tmp[0], tmp[1], tmp[2]);
	Plane p1 = Plane(v00, v10, v20);

	vh = mesh_->fv_iter(t1);
	tmp = mesh_->point(vh++);
	auto v01 = Vector3(tmp[0], tmp[1], tmp[2]);
	tmp = mesh_->point(vh++);
	auto v11 = Vector3(tmp[0], tmp[1], tmp[2]);
	tmp = mesh_->point(vh);
	auto v21 = Vector3(tmp[0], tmp[1], tmp[2]);
	Plane p2 = Plane(v01, v11, v21);

	Vector3 normal1 = p1.normal;
	Vector3 normal2 = p2.normal;

	qem += p1.distanceMatrix();
	qem += p2.distanceMatrix();
	
	if (mesh_->is_boundary(edge_index)) {
		tmp = mesh_->point(v0);
		Vector3 third_point = Vector3(tmp[0], tmp[1], tmp[2]) + normal1;
		tmp = mesh_->point(v0);
		auto tmp2 = mesh_->point(v1);
		Plane border_plane = Plane(Vector3(tmp[0], tmp[1], tmp[2]), Vector3(tmp2[0], tmp2[1], tmp2[2]), third_point);
		qem += border_plane.distanceMatrix();
	} else {
		Real angle = std::acos(normal1.dot(normal2));  // in radians
		Real alpha = std::cos(3.141592653589793238462 - angle);
		if (alpha > epsilon) {
			Vector3 normal_interpolated = (normal1 + normal2).normalized();
			tmp = mesh_->point(v0);
			Real d = normal_interpolated.dot(Vector3(tmp[0], tmp[1], tmp[2]));
			Plane new_plane(normal_interpolated, d);
			qem += alpha * new_plane.distanceMatrix();
		}
	}
	return qem;
}

Matrix44 MetricGenerator::computeInitialMergeMetric(TriMesh::VertexHandle vertex_index)
{
	TriMesh::Point tmp;
	Matrix44 qem = Matrix44::Zero();
	if (initialization_ == Midpoint) {
		for (TriMesh::VertexFaceIter vf_it = mesh_->vf_iter(vertex_index); vf_it->is_valid(); ++vf_it) {
			Vector3 midpoint = Vector3::Zero();

			for (TriMesh::FaceVertexIter fv_it = mesh_->fv_iter(*vf_it); fv_it.is_valid(); ++fv_it) {
				tmp = mesh_->point(*fv_it);
				midpoint += Vector3(tmp[0], tmp[1], tmp[2]);
			}
			midpoint /= 3;
			Matrix44 distance_point = Matrix44::Identity();
			distance_point.col(3) << -midpoint, 0;
			distance_point = distance_point.transpose() * distance_point;
			qem = mergeMatrices(qem, distance_point);
		}
		Matrix44 distance_point = Matrix44::Identity();
		tmp = mesh_->point(vertex_index);
		distance_point.col(3) << -Vector3(tmp[0], tmp[1], tmp[2]), 0;
		distance_point = distance_point.transpose() * distance_point;
		qem = mergeMatrices(qem, distance_point);
	} else {
		for (TriMesh::VertexFaceIter vf_it = mesh_->vf_iter(vertex_index); vf_it->is_valid(); ++vf_it) {
			auto vh = mesh_->fv_iter(*vf_it);
			tmp = mesh_->point(vh++);
			Vector3 v0 = Vector3(tmp[0], tmp[1], tmp[2]);
			tmp = mesh_->point(vh++);
			Vector3 v1 = Vector3(tmp[0], tmp[1], tmp[2]);
			tmp = mesh_->point(vh);
			Vector3 v2 = Vector3(tmp[0], tmp[1], tmp[2]);
			Plane p = Plane(v0, v1, v2);
			qem = mergeMax(qem, p.distanceMatrix());
		}

		for (TriMesh::VertexEdgeIter ve_it = mesh_->ve_iter(vertex_index); ve_it->is_valid(); ++ve_it) {
			TriMesh::HalfedgeHandle he = mesh_->halfedge_handle(*ve_it, 0);
			auto v0 = mesh_->to_vertex_handle(he);
			auto v1 = mesh_->from_vertex_handle(he);

			tmp = mesh_->point(v0);
			Vector3 position = Vector3(tmp[0], tmp[1], tmp[2]);
			tmp = mesh_->point(v1);
			Vector3 direction = (Vector3(tmp[0], tmp[1], tmp[2]) - position).normalized();
			Matrix44 distance_line = Matrix44::Zero();
			// Compute distance to line
			Matrix44 subtracted = Matrix44::Identity();
			subtracted.col(3) << -position, 0;
			distance_line += subtracted;
			Matrix44 dot_prod = Matrix44::Zero();
			dot_prod(0, 0) = direction(0);
			dot_prod(1, 1) = direction(1);
			dot_prod(2, 2) = direction(2);
			Matrix44 scalar_prod = Matrix44::Zero();
			scalar_prod.block(0, 0, 3, 3) << direction, direction, direction;
			distance_line -= scalar_prod * dot_prod * subtracted;
			distance_line = distance_line.transpose() * distance_line;
			qem = mergeMax(qem, distance_line);
		}

		Matrix44 distance_point = Matrix44::Identity();
		tmp = mesh_->point(vertex_index);
		distance_point.col(3) << -Vector3(tmp[0], tmp[1], tmp[2]), 0;
		distance_point = distance_point.transpose() * distance_point;
		qem = mergeMax(qem, distance_point);
	}
	return qem;
}

Matrix44 MetricGenerator::mergeMatrices(const Matrix44& a, const Matrix44& b)
{
	Matrix44 result = Matrix44::Zero();
	switch (metric_) {
		case MinimizedConstant:
			result = mergeMinConstant(a, b);
			break;
		case Diagonalization:
			result = mergeDiagonalization(a, b);
			break;
		case Average:
			result = mergeAverage(a, b);
			break;
		default:
			assert(false);
	}
	return result;
}

Matrix44 MetricGenerator::mergeMax(const Matrix44& a, const Matrix44& b)
{
	Matrix44 result = a + b;
	Eigen::Matrix<Real, 4, 3> E = Eigen::Matrix<Real, 4, 3>::Identity();
	Eigen::Matrix<Real, 4, 1> f(0, 0, 0, 1);

	Eigen::Matrix<Real, 3, 3> A_a = E.transpose() * a * E;
	Eigen::Matrix<Real, 3, 1> b_a = -(E.transpose() * a * f);
	Eigen::Matrix<Real, 3, 1> minimizer_a = A_a.ldlt().solve(b_a);
	Eigen::Matrix<Real, 4, 1> minimizer_4_a;
	minimizer_4_a << minimizer_a, 1;
	Real minimum_a = minimizer_4_a.transpose() * a * minimizer_4_a;

	Eigen::Matrix<Real, 3, 1> b_b = -(E.transpose() * b * f);
	Eigen::Matrix<Real, 3, 1> minimizer_b = A_a.ldlt().solve(b_b);
	Eigen::Matrix<Real, 4, 1> minimizer_4_b;
	minimizer_4_b << minimizer_b, 1;
	Real minimum_b = minimizer_4_b.transpose() * b * minimizer_4_b;

	Real constant_correction = std::min(minimum_a, minimum_b) - epsilon;
	if (constant_correction > 0) result(3, 3) -= constant_correction;
	return result;
}

Matrix44 MetricGenerator::mergeMinConstant(const Matrix44& a,
	const Matrix44& b)
{
	Matrix44 result = a + b;
	Eigen::Matrix<Real, 4, 3> E = Eigen::Matrix<Real, 4, 3>::Identity();
	Eigen::Matrix<Real, 4, 1> f(0, 0, 0, 1);
	Eigen::Matrix<Real, 3, 3> A = E.transpose() * result * E;
	Eigen::Matrix<Real, 3, 1> b_ = -(E.transpose() * result * f);
	Eigen::Matrix<Real, 3, 1> minimizer = A.ldlt().solve(b_);

	Eigen::Matrix<Real, 4, 1> minimizer_4;
	minimizer_4 << minimizer, 1;
	Real minimum_r = minimizer_4.transpose() * result * minimizer_4;
	Real minimum_a = minimizer_4.transpose() * a * minimizer_4;
	Real minimum_b = minimizer_4.transpose() * b * minimizer_4;

	Real constant_correction =
		minimum_r - std::min(minimum_a, minimum_b) - epsilon;
	if (constant_correction > 0) result(3, 3) -= constant_correction;

	// Debugging
	Eigen::SelfAdjointEigenSolver<Matrix44> es;
	es.compute(result);
	if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
		es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
		std::cout << "mergeMinConstant eigenvalues: "
			<< es.eigenvalues().transpose() << std::endl;
		std::cout << "correction: " << constant_correction << std::endl
			<< result << std::endl;
		es.compute(a + b);
		if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
			es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon)
			std::cout << "eigenvalues without subtract: "
			<< es.eigenvalues().transpose() << std::endl;
		else
			std::cout << "Fail introduced by correction" << std::endl;
	}
	return result;
}

Matrix44 MetricGenerator::mergeDiagonalization(const Matrix44& a,
	const Matrix44& b)
{
	// std::cout<<"merging "<<std::endl<<a<<std::endl<<b<<std::endl;
	// std::cout<<"a"<<std::endl<<a<<std::endl;
	// std::cout<<"b"<<std::endl<<b<<std::endl;

	Matrix44 result = a + b;
	// std::cout<<"result"<<std::endl<<result<<std::endl;

	Eigen::Matrix<Real, 4, 3> E = Eigen::Matrix<Real, 4, 3>::Identity();
	Eigen::Matrix<Real, 4, 1> f(0, 0, 0, 1);
	Eigen::Matrix<Real, 3, 3> A = E.transpose() * result * E;
	Eigen::Matrix<Real, 3, 1> b_ = -(E.transpose() * result * f);
	Eigen::Matrix<Real, 3, 1> minimizer = A.ldlt().solve(b_);

#if 0
	std::cout << "A:" << std::endl << A << std::endl;
	std::cout << "b: " << b_.transpose() << std::endl;
	std::cout << "min for transl " << minimizer.transpose() << std::endl;
	std::cout << "A*min " << (A*minimizer).transpose() << std::endl;
#endif

	Matrix44 translation = Matrix44::Identity();
	translation.col(3) << -minimizer, 1;
	Matrix44 translation_inv = Matrix44::Identity();
	translation_inv.col(3) << minimizer, 1;

	Matrix44 modified = translation_inv.transpose() * result * translation_inv;
	Eigen::Matrix<Real, 3, 3> block = modified.block(0, 0, 3, 3);
	Eigen::JacobiSVD<Eigen::Matrix<Real, 3, 3> > svd(
		block, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Matrix44 rotation_inv = Matrix44();
	rotation_inv << svd.matrixV(), Vector3::Zero(), Vector3::Zero().transpose(),
		1;

	Matrix44 transform = rotation_inv.transpose() * translation;
	Matrix44 transform_inv = translation_inv * rotation_inv;

	// std::cout<<"transform"<<std::endl<<transform<<std::endl;

	Matrix44 a_transformed = transform_inv.transpose() * a * transform_inv;
	Matrix44 b_transformed = transform_inv.transpose() * b * transform_inv;

	// std::cout<<"a_t"<<std::endl<<a_transformed<<std::endl;
	// std::cout<<"b_t"<<std::endl<<b_transformed<<std::endl;

	// Diagonalisation
	Vector3 a_diagonalised = Vector3::Zero();
	a_diagonalised(0) = std::sqrt(a_transformed(0, 0) * a_transformed(0, 0) +
		a_transformed(1, 0) * a_transformed(1, 0) +
		a_transformed(2, 0) * a_transformed(2, 0));
	a_diagonalised(1) = std::sqrt(a_transformed(0, 1) * a_transformed(0, 1) +
		a_transformed(1, 1) * a_transformed(1, 1) +
		a_transformed(2, 1) * a_transformed(2, 1));
	a_diagonalised(2) = std::sqrt(a_transformed(0, 2) * a_transformed(0, 2) +
		a_transformed(1, 2) * a_transformed(1, 2) +
		a_transformed(2, 2) * a_transformed(2, 2));
	a_diagonalised(3) = a_transformed(3, 3);

	Vector3 b_diagonalised = Vector3::Zero();
	b_diagonalised(0) = std::sqrt(b_transformed(0, 0) * b_transformed(0, 0) +
		b_transformed(1, 0) * b_transformed(1, 0) +
		b_transformed(2, 0) * b_transformed(2, 0));
	b_diagonalised(1) = std::sqrt(b_transformed(0, 1) * b_transformed(0, 1) +
		b_transformed(1, 1) * b_transformed(1, 1) +
		b_transformed(2, 1) * b_transformed(2, 1));
	b_diagonalised(2) = std::sqrt(b_transformed(0, 2) * b_transformed(0, 2) +
		b_transformed(1, 2) * b_transformed(1, 2) +
		b_transformed(2, 2) * b_transformed(2, 2));
	b_diagonalised(3) = b_transformed(3, 3);

	Vector3 linear_terms = Vector3::Zero();
	linear_terms(0) =
		std::max(std::abs(a_transformed(0, 3)), std::abs(b_transformed(0, 3)));
	linear_terms(1) =
		std::max(std::abs(a_transformed(1, 3)), std::abs(b_transformed(1, 3)));
	linear_terms(2) =
		std::max(std::abs(a_transformed(2, 3)), std::abs(b_transformed(2, 3)));
	// std::cout<<"a diag "<<a_diagonalised.transpose()<<std::endl;
	// std::cout<<"b diag "<<b_diagonalised.transpose()<<std::endl;
	// std::cout<<"linear "<<linear_terms.transpose()<<std::endl;
	Real l = 10;  // free parameter

	Matrix44 final_diagonalized = Matrix44::Zero();
	final_diagonalized(0, 0) = std::max(a_diagonalised(0), b_diagonalised(0)) +
		linear_terms(0) / (2 * l);
	final_diagonalized(1, 1) = std::max(a_diagonalised(1), b_diagonalised(1)) +
		linear_terms(1) / (2 * l);
	final_diagonalized(2, 2) = std::max(a_diagonalised(2), b_diagonalised(2)) +
		linear_terms(2) / (2 * l);
	final_diagonalized(3, 3) = std::max(a_diagonalised(3), b_diagonalised(3)) +
		linear_terms(0) * l / 2 + linear_terms(1) * l / 2 +
		linear_terms(2) * l / 2;
	// std::cout<<"final_d"<<std::endl<<final_diagonalized<<std::endl;

	Matrix44 final_retransformed =
		transform.transpose() * final_diagonalized * transform;
	Matrix44 ret_uncorr = final_retransformed;
	// std::cout<<"final_d*t"<<std::endl<<final_diagonalized*
	// transform<<std::endl;
	// std::cout<<"final "<<std::endl<<final_retransformed<<std::endl;

	// Find correct constant
	/*Matrix44 m = final_retransformed - a;
	Matrix44 copy_m1 = m;
	A = E.transpose() * m * E;
	b_ = -(E.transpose() * m * f);
	minimizer = A.colPivHouseholderQr().solve(b_);

	Eigen::Matrix<Real, 4, 1> minimizer_4;
	minimizer_4<<minimizer, 1;
	//std::cout<<"min (q-a) "<<std::endl<<minimizer_4<<std::endl;
  //	std::cout<<"q-a"<<std::endl<<m<<std::endl;
	Real m1 = minimizer_4.transpose() * m * minimizer_4;
  //	std::cout<<"m*min"<<std::endl<<m * minimizer_4<<std::endl;
  //	std::cout<<"m1 "<<m1<<std::endl;
	m = final_retransformed - b;
	Matrix44 copy_m2 = m;
	A = E.transpose() * m * E;
	b_ = -(E.transpose() * m * f);
	minimizer = A.colPivHouseholderQr().solve(b_);
	minimizer_4<<minimizer, 1;
  //	std::cout<<"min (q-b) "<<minimizer_4.transpose()<<std::endl;
  //	std::cout<<"q-a"<<std::endl<<m<<std::endl;
	Real m2 = minimizer_4.transpose() * m * minimizer_4;
  //	std::cout<<"m2 "<<m2<<std::endl;
  //	std::cout<<final_diagonalized(3,3)<<std::endl;
	final_retransformed(3,3) -= std::min(m1, m2);
  //	std::cout<<final_diagonalized(3,3)<<std::endl;
	*/
	// std::cout<<"corrected final "<<std::endl<<final_retransformed<<std::endl;

	// Debugging
	Eigen::SelfAdjointEigenSolver<Matrix44> es;
	es.compute(final_retransformed);
	Real epsilon = 0.001;
	if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
		es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
		std::cout << "Bad matrix eigenvalues: " << es.eigenvalues().transpose()
			<< std::endl
			<< "Metric:" << std::endl
			<< final_retransformed << std::endl
			<< "transform: " << transform
			<< std::endl;  //<<"Const correct "<<std::min(m1, m2)<<std::endl;
		es.compute(ret_uncorr);
		if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
			es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
			std::cout << "final retransformed is bad" << std::endl;
		} else
			std::cout << "const correct broke matrix" << std::endl;
		// std::cout<<copy_m1<<std::endl<<copy_m2<<std::endl;
	}
	return final_retransformed;
}

Matrix44 MetricGenerator::mergeAverage(const Matrix44& a, const Matrix44& b)
{
	Matrix44 result = (a + b) * 0.5;
	return result;
}

void MetricGenerator::cleanAndRenumber()
{
	/*
	switch (metric_) {
		case ClassicQEM:
			shrinkIndexedArray(&qems_, mesh_->deleted_vertices_);
			break;
		case ModifiedQEM:
			shrinkIndexedArray(&vertices_qem_, mesh_->deleted_vertices_);
			shrinkIndexedArray(&edges_qem_, mesh_->deleted_edges_);
			shrinkIndexedArray(&triangles_qem_, mesh_->deleted_triangles_);
			break;
		case MinimizedConstant:
		case Diagonalization:
		case Average:
			shrinkIndexedArray(&qems_merge_, mesh_->deleted_vertices_);
			break;
		default:
			break;
	}
	*/
}

void MetricGenerator::shrinkIndexedArray(MatrixArray* array,
	std::stack<Index> deleted_indices)
{
	MatrixArray new_array;
	std::vector<Index> deleted_indices_sorted;
	deleted_indices_sorted.reserve(deleted_indices.size());
	while (deleted_indices.size() > 0) {
		deleted_indices_sorted.push_back(deleted_indices.top());
		deleted_indices.pop();
	}
	std::sort(deleted_indices_sorted.begin(), deleted_indices_sorted.end());

	Index next_index = 0;
	unsigned int deleted_i = 0;
	if (array->size() <= deleted_indices_sorted.size()) {
		// TODO find cause for too many deleted indices
		std::cout << "What" << std::endl;
		array->swap(new_array);
		return;
	}
	unsigned int number_valid = array->size() - deleted_indices_sorted.size();
	for (unsigned int i = 0; i < number_valid; ++i) {
		while (deleted_i < deleted_indices_sorted.size() &&
			next_index == deleted_indices_sorted[deleted_i]) {
			next_index++;
			deleted_i++;
		}
		new_array.push_back((*array)[next_index]);
		next_index++;
	}
	array->swap(new_array);
}
}  // namespace boundingmesh