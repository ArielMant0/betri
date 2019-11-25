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

#include "Decimator.hh"

#include <cmath>
#include <limits>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "SimpleOptimizer.hh"
// for debugging
#include <Eigen/Eigenvalues>

namespace boundingmesh
{
Decimator::Decimator(DecimationDirection direction)
	: direction_(direction), optimizer_(new SimpleOptimizer)
{}

Decimator::Decimator(std::unique_ptr<OptimizerInterface> optimizer,
	DecimationDirection direction)
	: target_vertices_(default_target_vertices),
	maximum_error_(default_maximum_error),
	direction_(direction),
	optimizer_(std::move(optimizer))
{}

Decimator::~Decimator()
{}

Real Decimator::currentError()
{
	return current_error_;
}

Real Decimator::nextError()
{
	if (queue_.size() == 0) return 0;
	const EdgeContraction& contraction = queue_.first();
	return contraction.cost();
}

void Decimator::setTargetVertices(int target_vertices)
{
	target_vertices_ = target_vertices;
	target_vertices_used_ = true;
}

void Decimator::unsetTargetVertices()
{
	target_vertices_used_ = false;
}

void Decimator::setMaximumError(Real maximum_error)
{
	maximum_error_ = maximum_error;
	maximum_error_used_ = true;
}

void Decimator::unsetMaximumError()
{
	maximum_error_used_ = false;
}

void Decimator::setDirection(DecimationDirection direction)
{
	direction_ = direction;

	if (result_mesh_ != NULL) recomputeQueue();
}

void Decimator::setMetric(Metric metric)
{
	metric_generator_.setMetric(metric);

	if (result_mesh_ != NULL) recomputeQueue();
}

void Decimator::setInitialization(Initialization initialization)
{
	metric_generator_.setInitialization(initialization);

	if (result_mesh_ != NULL) recomputeQueue();
}

void Decimator::setMesh(const TriMesh& mesh)
{
	result_mesh_ = std::make_shared<TriMesh>(mesh);
	metric_generator_.setMesh(result_mesh_);

	recomputeQueue();
}

::std::shared_ptr<TriMesh> Decimator::getMesh()
{
	return result_mesh_;
}

void Decimator::recomputeQueue()
{
	queue_ = ContractionQueue();
	for (unsigned int i = 0; i < result_mesh_->n_edges(); ++i)
		queue_.insert(computeEdgeContraction(result_mesh_->edge_handle(i)));
}

::std::shared_ptr<TriMesh> Decimator::compute(ComputeCallback callback)
{
	if (queue_.size() == 0) {
		std::cout << "Bad mesh, no edges in list" << std::endl;
		result_mesh_.reset(new TriMesh());
		return result_mesh_;
	}

	if (!target_vertices_used_ && !maximum_error_used_) 
		return result_mesh_;

	// Greedy decimation, repeatedly contract best edge
	while ((queue_.size() > 0) &&
		(!target_vertices_used_ ||
			result_mesh_->n_vertices() > target_vertices_) &&
			(!maximum_error_used_ || queue_.first().cost() < maximum_error_)) {
		if (callback != NULL)
			callback(result_mesh_->n_vertices(), queue_.first().cost());
		const EdgeContraction contraction = queue_.first();
		executeEdgeContraction(contraction);
	}
	cleanAndRenumber();
	return result_mesh_;
}

::std::shared_ptr<TriMesh> Decimator::doContractions(unsigned int n)
{
	if (queue_.size() <= n) {
		std::cerr << "Warning: Cannot perform this many contractions" << std::endl;
		return ::std::make_shared<TriMesh>();
	}
	for (unsigned int i = 0; i < n; ++i) {
		const EdgeContraction& contraction = queue_.first();
		if (maximum_error_used_ && contraction.cost() > maximum_error_) 
			break;
		executeEdgeContraction(contraction);
	}
	cleanAndRenumber();
	return result_mesh_;
}

void Decimator::executeEdgeContraction(const EdgeContraction& contraction)
{
	current_error_ = contraction.cost();

	TriMesh::HalfedgeHandle edge = result_mesh_->halfedge_handle(contraction.edge(), 0);
	metric_generator_.contractEdge(contraction.edge());
	auto v0 = result_mesh_->to_vertex_handle(edge);
	auto v1 = result_mesh_->from_vertex_handle(edge);

	// First, remove edge, vertices and surrounding triangles
	std::set<TriMesh::EdgeHandle> edges_to_remove;
	std::vector<TriMesh::VertexHandle*> hole_border;
	collectRemovalData(v0, v1, edges_to_remove, hole_border);
	collectRemovalData(v1, v0, edges_to_remove, hole_border);

	for (std::set<TriMesh::EdgeHandle>::iterator it = edges_to_remove.begin(); it != edges_to_remove.end(); ++it) {
		queue_.remove(*it);
	}

	result_mesh_->delete_vertex(v0);
	result_mesh_->delete_vertex(v1);

	// Then insert new vertex and connect it correctly
	TriMesh::VertexHandle new_vertex_index = result_mesh_->add_vertex(contraction.new_point());
	for (unsigned int i = 0; i < hole_border.size(); ++i) {
		result_mesh_->add_face(new_vertex_index, hole_border[i][0], hole_border[i][1]);
		delete[] hole_border[i];
	}

	//const TriMesh::Vertex& new_vertex = result_mesh_->vertex(new_vertex_index);
	std::set<TriMesh::EdgeHandle> edges_to_add;
	for (TriMesh::VertexFaceIter vf_it = result_mesh_->vf_iter(new_vertex_index); vf_it.is_valid(); ++vf_it) {
		for (TriMesh::FaceEdgeIter fe_it = result_mesh_->fe_iter(*vf_it); fe_it.is_valid(); ++fe_it) {
			edges_to_add.insert(fe_it);
		}
	}
	for (std::set<TriMesh::EdgeHandle>::iterator it = edges_to_add.begin(); it != edges_to_add.end(); ++it) {
		queue_.insert(computeEdgeContraction(*it));
	}
}

void Decimator::collectRemovalData(TriMesh::VertexHandle vertex_index,
	TriMesh::VertexHandle other_index,
	std::set<TriMesh::EdgeHandle>& edges_to_remove,
	std::vector<TriMesh::VertexHandle*>& hole_border)
{
	const TriMesh::Vertex& vertex = result_mesh_->vertex(vertex_index);
	// Collects data to remove a patch of triangles around a vertex.
	// Is called for both vertices of an edge, with vertex_index and other_index
	// swapped
	for (TriMesh::VertexFaceIter vf_it = result_mesh_->vf_iter(vertex_index); vf_it.is_valid(); ++vf_it) {
		const TriMesh::Face& triangle = result_mesh_->face(vf_it);
		bool is_shared_triangle = false;
		for (TriMesh::FaceEdgeIter fe_it = result_mesh_->fe_iter(*vf_it); fe_it.is_valid(); ++fe_it) {
			edges_to_remove.insert(fe_it);

			TriMesh::HalfedgeHandle edge = result_mesh_->halfedge_handle(fe_it, 0);
			auto v0 = result_mesh_->to_vertex_handle(edge);

			if (v0 == other_index)
				is_shared_triangle = true;
		}
		
		if (!is_shared_triangle) {

			auto vh = result_mesh_->fv_iter(*vf_it);
			auto v0 = *vh; vh++;
			auto v1 = *vh; vh++;
			auto v2 = *vh;

			TriMesh::VertexHandle* border_edge = new TriMesh::VertexHandle[2];
			if (v0 == vertex_index) {
				border_edge[0] = v1;
				border_edge[1] = v2;
			} else if (v1 == vertex_index) {
				border_edge[0] = v2;
				border_edge[1] = v0;
			} else if (v2 == vertex_index) {
				border_edge[0] = v0;
				border_edge[1] = v1;
			}
			hole_border.push_back(border_edge);
		}
	}
}

EdgeContraction Decimator::computeEdgeContraction(TriMesh::EdgeHandle edge_index)
{
	TriMesh::HalfedgeHandle hEdge = result_mesh_->halfedge_handle(edge_index, 0);
	auto v0 = result_mesh_->to_vertex_handle(hEdge);
	auto v1 = result_mesh_->from_vertex_handle(hEdge);

	const TriMesh::Vertex& vertex_1 = result_mesh_->vertex(v0);
	const TriMesh::Vertex& vertex_2 = result_mesh_->vertex(v1);

	Matrix44 qem = metric_generator_.getErrorMetric(edge_index);
	TriMesh::Point new_point = TriMesh::Point();

#ifndef NDEBUG
	// Debugging: error matrices shoud be positive semidefinite
	Eigen::SelfAdjointEigenSolver<Matrix44> es;
	es.compute(qem);
	if (es.eigenvalues()(0) < -epsilon || es.eigenvalues()(1) < -epsilon ||
		es.eigenvalues()(2) < -epsilon || es.eigenvalues()(3) < -epsilon) {
		std::cerr << "Warning: Bad metric eigenvalues: "
			<< es.eigenvalues().transpose() << std::endl
			<< "Metric:" << std::endl
			<< qem << std::endl;
	}
#endif

	// Detect edges with one vertex on a border edge
	int border_vertices = 0;
	for (TriMesh::VertexEdgeIter ve_it = result_mesh_->ve_iter(v0); ve_it.is_valid(); ++ve_it) {
		if (result_mesh_->is_boundary(*ve_it)) {
			new_point = result_mesh_->point(v0);
			border_vertices++;
			break;
		}
	}
	for (TriMesh::VertexEdgeIter ve_it = result_mesh_->ve_iter(v1); ve_it.is_valid(); ++ve_it) {
		if (result_mesh_->is_boundary(*ve_it)) {
			new_point = result_mesh_->point(v1);
			border_vertices++;
			break;
		}
	}

	if (border_vertices == 2 && result_mesh_->is_boundary(edge_index)) {
		/* Edge connects two borders, collapsing this would result in bad topology
		Current situation:
				___________(Border)
				...../|....
				..../.|....
				.../..|<--This edge
				...\..|....
				....\.|....
				_____\|____(Border)
		Would result in:
				___      __(Border)
				...\    /..
				....\  /...
				.....\/<--New Point
				...../\....
				..../  \...
				___/    \__(Border)
		*/
		return EdgeContraction(edge_index, TriMesh::Point(),
			std::numeric_limits<Real>::max(), Matrix44::Zero());
	}

	Vector3 tmp_vec3 = Vector3(new_point[0], new_point[1], new_point[2]);
	if (direction_ == Any) {
		if (border_vertices == 0 || border_vertices == 2) {
#ifndef NDEBUG
			Real unused_cost;
#endif
			assert(optimizer_->optimize(qem, {}, false, &tmp_vec3, &unused_cost));
			new_point = TriMesh::Point(tmp_vec3[0], tmp_vec3[1], tmp_vec3[2]);
		}
		// For border_vertices == 1 the new point is already found

		Vector4 new_point_homogeneous;
		new_point_homogeneous << tmp_vec3, 1;
		Real cost = new_point_homogeneous.transpose() * qem * new_point_homogeneous;
		return EdgeContraction(edge_index, new_point, cost, qem);
	}

	// collect constraint planes
	std::vector<Plane> constraints;
	for (TriMesh::VertexFaceIter vf_it = result_mesh_->vf_iter(v0); vf_it.is_valid(); ++vf_it) {
		auto vh = result_mesh_->fv_iter(*vf_it);
		auto tmp = result_mesh_->point(vh++);
		auto v0 = Vector3(tmp[0], tmp[1],tmp[2]);
		tmp = result_mesh_->point(vh++);
		auto v1 = Vector3(tmp[0], tmp[1], tmp[2]);
		tmp = result_mesh_->point(vh);
		auto v2 = Vector3(tmp[0], tmp[1], tmp[2]);
		Plane p = Plane(v0, v1, v2);
		constraints.push_back(p);
	}

	for (TriMesh::VertexFaceIter vf_it = result_mesh_->vf_iter(v1); vf_it.is_valid(); ++vf_it) {
		bool shared_triangle = false;
		for (TriMesh::FaceVertexIter fv_it = result_mesh_->fv_iter(vf_it); fv_it.is_valid(); ++fv_it) {
			if (*fv_it == v0) {
				shared_triangle = true;
				break;
			}
		}
		if (!shared_triangle) {
			auto vh = result_mesh_->fv_iter(*vf_it);
			auto tmp = result_mesh_->point(vh++);
			auto v0 = Vector3(tmp[0], tmp[1], tmp[2]);
			tmp = result_mesh_->point(vh++);
			auto v1 = Vector3(tmp[0], tmp[1], tmp[2]);
			tmp = result_mesh_->point(vh);
			auto v2 = Vector3(tmp[0], tmp[1], tmp[2]);
			Plane p = Plane(v0, v1, v2);
			constraints.push_back(p);
		}
	}

	// if one point is on the border, check if decimation is allowed
	if (border_vertices == 1) {
		bool valid_solution = true;
		Vector4 new_point_homogeneous;
		new_point_homogeneous << tmp_vec3, 1;
		for (unsigned int i = 0; i < constraints.size(); ++i) {
			if (direction_ == Outward &&
				constraints[i].vector().transpose() * new_point_homogeneous +
				epsilon <
				0) {
				valid_solution = false;
				break;
			} else if (direction_ == Inward &&
				constraints[i].vector().transpose() * new_point_homogeneous -
				epsilon >
				0) {
				valid_solution = false;
				break;
			}
		}
		if (valid_solution) {
			Real cost = new_point_homogeneous.transpose() * qem * new_point_homogeneous;
			return EdgeContraction(edge_index, new_point, cost, qem);
		} else
			return EdgeContraction(edge_index, TriMesh::Point(),
				std::numeric_limits<Real>::max(),
				Matrix44::Zero());
	}

	Real minimal_cost;
	bool found = optimizer_->optimize(
		qem, constraints, (direction_ == DecimationDirection::Outward),
		&tmp_vec3, &minimal_cost
	);
	new_point = TriMesh::Point(tmp_vec3[0], tmp_vec3[1], tmp_vec3[2]);

	if (found)
		return EdgeContraction(edge_index, new_point, minimal_cost, qem);
	else
		return EdgeContraction(edge_index, TriMesh::Point(),
			std::numeric_limits<Real>::max(), Matrix44::Zero());
}

void Decimator::cleanAndRenumber()
{
	ContractionQueue new_queue;
	int next_index = 0;
	for (std::set<ContractionIndex>::iterator it = queue_.indices_.begin(); it != queue_.indices_.end(); ++it) {
		/*
		The iteration over the queue_.indices_ set is ordered by index.
		Since the indices of deleted edges are missing, we can just put the position
		in the list as new index.
		This changes the edge indices just like Mesh::cleanAndRenumber
		Example:
		Set of indices: {1, 3, 5, 6, 7}
		Renumbered: 	{1, 2, 3, 4, 5}
		*/
		ContractionIterator contraction_it = it->contraction();
		new_queue.insert(EdgeContraction(
			TriMesh::EdgeHandle(next_index), contraction_it->new_point(),
			contraction_it->cost(),
			contraction_it->qem())
		);
		next_index++;
	}
	swap(queue_, new_queue);
	metric_generator_.cleanAndRenumber();
	result_mesh_->garbage_collection();
}

}  // namespace boundingmesh