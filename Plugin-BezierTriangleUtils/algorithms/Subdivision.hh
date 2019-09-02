#pragma once

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

namespace betri
{

template <typename MeshT>
class Subdivision
{
public:

	typedef typename MeshT::Point    Point;
	typedef typename MeshT::EdgeIter EdgeIter;
	typedef typename MeshT::VertexIter VertexIter;
	typedef typename MeshT::HalfedgeHandle HalfedgeHandle;
	typedef typename MeshT::VertexHandle VertexHandle;
	typedef typename MeshT::VertexOHalfedgeIter VertexOHalfedgeIter;
	typedef typename MeshT::VertexVertexIter VertexVertexIter;

	Subdivision(MeshT & _mesh) : m_mesh(_mesh)
	{
		m_mesh.add_property(vp_newPosition);
		m_mesh.add_property(ep_newPosition);
		m_mesh.add_property(ep_flagRed);
	}

	~Subdivision()
	{
		m_mesh.remove_property(vp_newPosition);
		m_mesh.remove_property(ep_newPosition);
		m_mesh.remove_property(ep_flagRed);
	}

	void loop(unsigned int steps=1)
	{
		HalfedgeHandle heHandle, nheHandle;
		Point newPosition;

		for (unsigned int i = 0; i < steps; ++i) {
			// INSERT CODE:
			// 1) clear all edge flags
			// 2) compute new positions for existing vertices (but don't apply them yet)
			// 3) compute new positions for edge vertices
			// 4) perform edge splits (2-to-4 splits) and compute positions of new vertices
			// 5) perform edge flips of red edges if both neighboring triangles contain another red edge too
			// 6) copy over new vertex positions, but only for OLD vertices

			//////////////////////////////////
			// clear all edge flags

			auto eEnd = m_mesh.edges_end();
			for (auto eIter = m_mesh.edges_begin(); eIter != eEnd; ++eIter) {
				m_mesh.property(ep_flagRed, *eIter) = false;
			}


			//////////////////////////////////
			// compute new positions for existing
			// vertices (but don't apply them yet)

			auto vEnd = m_mesh.vertices_end();
			for (auto vIter = m_mesh.vertices_begin(); vIter != vEnd; ++vIter) {

				newPosition = Point(0.0, 0.0, 0.0);

				double invValence = 1.0 / double(m_mesh.valence(*vIter));
				double beta = 0.375 + (0.25 * cos(2.0 * M_PI * invValence));
				beta = 0.625 - beta * beta;

				auto vvEnd = m_mesh.vv_end(*vIter);
				for (auto vvIter = m_mesh.vv_begin(*vIter); vvIter != vvEnd; ++vvIter) {
					newPosition += m_mesh.point(*vvIter);
				}

				newPosition *= beta * invValence;
				newPosition += m_mesh.point(*vIter) * (1.0 - beta);

				// store new position
				m_mesh.property(vp_newPosition, *vIter) = newPosition;
			}


			//////////////////////////////////
			// compute new positions for edge vertices

			for (auto eIter = m_mesh.edges_begin(); eIter != eEnd; ++eIter) {
				// compute position of edge vertex
				heHandle = m_mesh.halfedge_handle(*eIter, 0);
				nheHandle = m_mesh.next_halfedge_handle(heHandle);
				newPosition = m_mesh.point(m_mesh.to_vertex_handle(heHandle))*0.375;
				newPosition += m_mesh.point(m_mesh.to_vertex_handle(nheHandle))*0.125;

				heHandle = m_mesh.halfedge_handle(*eIter, 1);
				nheHandle = m_mesh.next_halfedge_handle(heHandle);
				newPosition += m_mesh.point(m_mesh.to_vertex_handle(heHandle))*0.375;
				newPosition += m_mesh.point(m_mesh.to_vertex_handle(nheHandle))*0.125;

				m_mesh.property(ep_newPosition, *eIter) = newPosition;
			}


			//////////////////////////////////
			// perform edge splits (2-to-4 splits)
			// and compute positions of new vertices

			for (auto eIter = m_mesh.edges_begin(); eIter != eEnd; ++eIter) {
				// find vertices opposite to current edge
				VertexHandle vHandle1, vHandle2;

				heHandle = m_mesh.halfedge_handle(*eIter, 0);
				heHandle = m_mesh.next_halfedge_handle(heHandle);
				vHandle1 = m_mesh.to_vertex_handle(heHandle);

				heHandle = m_mesh.halfedge_handle(*eIter, 1);
				heHandle = m_mesh.next_halfedge_handle(heHandle);
				vHandle2 = m_mesh.to_vertex_handle(heHandle);

				// split edge
				VertexHandle newVertex;
				newVertex = m_mesh.add_vertex(m_mesh.property(ep_newPosition, *eIter));
				m_mesh.split(*eIter, newVertex);

				auto vohEnd = m_mesh.voh_end(newVertex);
				// mark two new edges as "red"
				for (auto vohIter = m_mesh.voh_begin(newVertex); vohIter != vohEnd; ++vohIter) {
					if (m_mesh.to_vertex_handle(*vohIter) == vHandle1 ||
						m_mesh.to_vertex_handle(*vohIter) == vHandle2) {
						// this is one of the new edges
						m_mesh.property(
							ep_flagRed,
							m_mesh.edge_handle(*vohIter)) = true;
					}
				}
			}


			///////////////////////////////////////
			// perform edge flips of red edges
			// if both neighboring triangles contain another red edge too

			eEnd = m_mesh.edges_end();
			for (auto eIter = m_mesh.edges_begin(); eIter != eEnd; ++eIter) {

				if (m_mesh.property(ep_flagRed, *eIter)) {

					heHandle = m_mesh.halfedge_handle(*eIter, 0);
					heHandle = m_mesh.next_halfedge_handle(heHandle);
					nheHandle = m_mesh.next_halfedge_handle(heHandle);

					if (!m_mesh.property(ep_flagRed, m_mesh.edge_handle(heHandle)) &&
						!m_mesh.property(ep_flagRed, m_mesh.edge_handle(nheHandle))) {
						// both edges are not flagged
						continue;
					}

					heHandle = m_mesh.halfedge_handle(*eIter, 1);
					heHandle = m_mesh.next_halfedge_handle(heHandle);
					nheHandle = m_mesh.next_halfedge_handle(heHandle);

					if (!m_mesh.property(ep_flagRed, m_mesh.edge_handle(heHandle)) &&
						!m_mesh.property(ep_flagRed, m_mesh.edge_handle(nheHandle))) {
						// both edges are not flagged
						continue;
					}

					// there are red edges in both adjacent triangles
					// => flip edge
					m_mesh.flip(*eIter);
				}
			}


			//////////////////////////////////
			// copy over new vertex positions, but only for OLD vertices

			for (auto vIter = m_mesh.vertices_begin(); vIter != vEnd; ++vIter) {
				m_mesh.set_point(*vIter, m_mesh.property(vp_newPosition, *vIter));
			}
		}
	}

private:

	BezierTMesh &m_mesh;

	OpenMesh::VPropHandleT<Point> vp_newPosition;
	OpenMesh::EPropHandleT<Point> ep_newPosition;
	OpenMesh::EPropHandleT<bool> ep_flagRed;
};

} // namespace betri