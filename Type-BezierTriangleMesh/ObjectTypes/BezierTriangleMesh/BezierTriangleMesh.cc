#pragma once

//== INCLUDES =================================================================

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

#include <ACG/Scenegraph/StatusNodesT.hh>
#include <ACG/QtScenegraph/QtTranslationManipulatorNode.hh>


BTMeshObject::BTMeshObject(const BTMeshObject &_object) : MeshObject<BezierTMesh>(_object) {}


BTMeshObject::BTMeshObject(DataType _typeId) : MeshObject<BezierTMesh>(_typeId) {}


BTMeshObject::~BTMeshObject() {}


BaseObject* BTMeshObject::copy()
{
	BTMeshObject *object = new BTMeshObject(*this);
	return object;
}

/// Refine picking on triangle meshes
ACG::Vec3d BTMeshObject::refinePick(
	ACG::SceneGraph::PickTarget _pickTarget,
	const ACG::Vec3d _hitPoint,
	const ACG::Vec3d _start ,
	const ACG::Vec3d _dir,
	const unsigned int _targetIdx
) {
	if (_pickTarget == ACG::SceneGraph::PICK_FACE) {
		// get picked face handle
		BezierTMesh::FaceHandle fh = mesh()->face_handle(_targetIdx);

		BezierTMesh::FaceVertexIter fv_it = mesh()->fv_begin(fh);

		// Get vertices of the face
		ACG::Vec3d p1 = mesh()->point(*fv_it);
		++fv_it;

		ACG::Vec3d p2 = mesh()->point(*fv_it);
		++fv_it;

		ACG::Vec3d p3 = mesh()->point(*fv_it);
		++fv_it;

		ACG::Vec3d hitpointNew = _hitPoint;

		BezierTMesh::Scalar t,u,v;
		if (ACG::Geometry::triangleIntersection(_start, _dir, p1, p2, p3, t, u, v)) {
			hitpointNew = _start + t * _dir;
		}

		return hitpointNew;
	}

	if (_pickTarget == ACG::SceneGraph::PICK_EDGE) {
		// get picked edge handle
		BezierTMesh::EdgeHandle eh = mesh()->edge_handle(_targetIdx);
		if(eh.is_valid()) {
			BezierTMesh::HalfedgeHandle heh = mesh()->halfedge_handle(eh,0);

			//get vertices of the edge
			BezierTMesh::VertexHandle vhbegin = mesh()->to_vertex_handle(heh);
			BezierTMesh::VertexHandle vhend = mesh()->from_vertex_handle(heh);
			ACG::Vec3d edgeStart = mesh()->point(vhbegin);
			ACG::Vec3d edgeEnd = mesh()->point(vhend);

			//retrieve the point on the edge that is closest to the backprojected hitpoint
			ACG::Vec3d hitPointNew;
			ACG::Geometry::distPointLineSquared(_hitPoint,edgeStart,edgeEnd,&hitPointNew);

			return hitPointNew;
		}
	}

	if (_pickTarget == ACG::SceneGraph::PICK_VERTEX) {
		// get picked vertex handle
		BezierTMesh::VertexHandle vh = mesh()->vertex_handle(_targetIdx);
		if(vh.is_valid()) {
			ACG::Vec3d hitpointNew = mesh()->point(vh);
			//just return the vertex position
			return hitpointNew;
		}
	}

	return _hitPoint;
}

//=============================================================================
