#pragma once

#include <ObjectTypes/TriangleMesh/TriangleMesh.hh>
#include <Type-OpenMesh/ObjectTypes/TriangleMesh/TriangleMeshTypes.hh>
#include <OpenFlipper/libs_required/ACG/Math/BezierCurveT.hh>

struct BezierTriangleTraits : public TriTraits
{
	EdgeTraits{
		size_t curveIndex;
	};

	FaceTraits{
		size_t bezierIndex;
	};
};

using BezierTriangleMesh = OpenMesh::TriMesh_ArrayKernelT<BezierTriangleTraits>;

class BezierTMesh
{
public:

	using Point = BezierTriangleMesh::Point;
	using Scalar = BezierTriangleMesh::Scalar;
	using Curve = ACG::BezierCurveT<Point>;

	BezierTMesh() {}

	BezierTMesh(TriMesh &mesh)
	{
		m_curves.reserve(mesh.n_edges());
		m_faces.reserve(mesh.n_faces());

		size_t index = 0;
		// copy connectivity from given triangle mesh
		m_mesh.assign_connectivity(mesh);
		// add a bezier curve for each existing edge
		for (auto eIt = m_mesh.edges_begin(); eIt != m_mesh.edges_end(); ++eIt) {
			m_mesh.data(*eIt).curveIndex = index++;
			const auto he1 = m_mesh.halfedge_handle(*eIt, 0);
			const auto he2 = m_mesh.halfedge_handle(*eIt, 1);
			const auto start = m_mesh.point(m_mesh.to_vertex_handle(he1));
			const auto end = m_mesh.point(m_mesh.to_vertex_handle(he2));
			// quadratic bezier curve, TODO
			Curve curve;
			curve.push_back(start);
			curve.push_back(start * 0.5f + end * 0.5f);
			curve.push_back(end);
			m_curves.push_back(curve);
		}
		index = 0;
		// add bezier triangles
		for (auto fIt = m_mesh.faces_begin(); fIt != m_mesh.faces_end(); ++fIt) {
			m_mesh.data(*fIt).bezierIndex = index++;
			auto efIt = m_mesh.fe_begin(*fIt);
			m_faces.push_back({
				m_mesh.data(*efIt).curveIndex,
				m_mesh.data(*(++efIt)).curveIndex,
				m_mesh.data(*(++efIt)).curveIndex
			});
		}
	}

private:

	// actual mesh
	BezierTriangleMesh m_mesh;
	// bezier curves that make up the bezier triangles
	std::vector<Curve> m_curves;
	// which curves make up a face
	std::vector<std::array<size_t, 3>> m_faces;

};