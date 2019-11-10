#ifndef QUICKHULL_HPP_
#define QUICKHULL_HPP_
#include <deque>
#include <vector>
#include <array>
#include <limits>
#include "Structs/Vector3.hpp"
#include "Structs/Plane.hpp"
#include "Structs/Pool.hpp"
#include "Structs/Mesh.hpp"
#include "ConvexHull.hpp"
#include "HalfEdgeMesh.hpp"
#include "MathUtils.hpp"

/*
 * Implementation of the 3D QuickHull algorithm by Antti Kuukka
 *
 * No copyrights. What follows is 100% Public Domain.
 *
 *
 *
 * INPUT:  a list of points in 3D space (for example, vertices of a 3D mesh)
 *
 * OUTPUT: a ConvexHull object which provides vertex and index buffers of the generated convex hull as a triangle mesh.
 *
 *
 *
 * The implementation is thread-safe if each thread is using its own QuickHull object.
 *
 *
 * SUMMARY OF THE ALGORITHM:
 *         - Create initial simplex (tetrahedron) using extreme points. We have four faces now and they form a convex mesh M.
 *         - For each point, assign them to the first face for which they are on the positive side of (so each point is assigned to at most
 *           one face). Points inside the initial tetrahedron are left behind now and no longer affect the calculations.
 *         - Add all faces that have points assigned to them to Face Stack.
 *         - Iterate until Face Stack is empty:
 *              - Pop topmost face F from the stack
 *              - From the points assigned to F, pick the point P that is farthest away from the plane defined by F.
 *              - Find all faces of M that have P on their positive side. Let us call these the "visible faces".
 *              - Because of the way M is constructed, these faces are connected. Solve their horizon edge loop.
 *				- "Extrude to P": Create new faces by connecting P with the points belonging to the horizon edge. Add the new faces to M and remove the visible
 *                faces from M.
 *              - Each point that was assigned to visible faces is now assigned to at most one of the newly created faces.
 *              - Those new faces that have points assigned to them are added to the top of Face Stack.
 *          - M is now the convex hull.
 *
 * TO DO:
 *  - Implement a proper 2D QuickHull and use that to solve the degenerate 2D case (when all the points lie on the same plane in 3D space).
 * */

namespace quickhull {

	struct DiagnosticsData {
		size_t m_failedHorizonEdges; // How many times QuickHull failed to solve the horizon edge. Failures lead to degenerated convex hulls.

		DiagnosticsData() : m_failedHorizonEdges(0) { }
	};

	template<typename FloatType>
	//FloatType defaultEps();

	//template<>
	float defaultEps()
	{
		return 0.0001f;
	}
	/*
	template<>
	double defaultEps()
	{
		return 0.0000001;
	}*/

	template<typename FloatType>
	class QuickHull {
		using vec3 = Vector3<FloatType>;

		FloatType m_epsilon, m_epsilonSquared, m_scale;
		bool m_planar;
		std::vector<vec3> m_planarPointCloudTemp;
		VertexDataSource<FloatType> m_vertexData;
		MeshBuilder<FloatType> m_mesh;
		std::array<size_t,6> m_extremeValues;
		DiagnosticsData m_diagnostics;

		// Temporary variables used during iteration process
		std::vector<size_t> m_newFaceIndices;
		std::vector<size_t> m_newHalfEdgeIndices;
		std::vector< std::unique_ptr<std::vector<size_t>> > m_disabledFacePointVectors;
		std::vector<size_t> m_visibleFaces;
		std::vector<size_t> m_horizonEdges;
		struct FaceData {
			size_t m_faceIndex;
			size_t m_enteredFromHalfEdge; // If the face turns out not to be visible, this half edge will be marked as horizon edge
			FaceData(size_t fi, size_t he) : m_faceIndex(fi),m_enteredFromHalfEdge(he) {}
		};
		std::vector<FaceData> m_possiblyVisibleFaces;
		std::deque<size_t> m_faceList;

		// Create a half edge mesh representing the base tetrahedron from which the QuickHull iteration proceeds. m_extremeValues must be properly set up when this is called.
		void setupInitialTetrahedron()
		{
			const size_t vertexCount = m_vertexData.size();

			// If we have at most 4 points, just return a degenerate tetrahedron:
			if (vertexCount <= 4) {
				size_t v[4] = { 0,std::min((size_t)1,vertexCount - 1),std::min((size_t)2,vertexCount - 1),std::min((size_t)3,vertexCount - 1) };
				const Vector3<FloatType> N = mathutils::getTriangleNormal(m_vertexData[v[0]], m_vertexData[v[1]], m_vertexData[v[2]]);
				const Plane<FloatType> trianglePlane(N, m_vertexData[v[0]]);
				if (trianglePlane.isPointOnPositiveSide(m_vertexData[v[3]])) {
					std::swap(v[0], v[1]);
				}
				return m_mesh.setup(v[0], v[1], v[2], v[3]);
			}

			// Find two most distant extreme points.
			FloatType maxD = m_epsilonSquared;
			std::pair<size_t, size_t> selectedPoints;
			for (size_t i = 0;i < 6;i++) {
				for (size_t j = i + 1;j < 6;j++) {
					const FloatType d = m_vertexData[m_extremeValues[i]].getSquaredDistanceTo(m_vertexData[m_extremeValues[j]]);
					if (d > maxD) {
						maxD = d;
						selectedPoints = { m_extremeValues[i],m_extremeValues[j] };
					}
				}
			}
			if (maxD == m_epsilonSquared) {
				// A degenerate case: the point cloud seems to consists of a single point
				return m_mesh.setup(0, std::min((size_t)1, vertexCount - 1), std::min((size_t)2, vertexCount - 1), std::min((size_t)3, vertexCount - 1));
			}
			assert(selectedPoints.first != selectedPoints.second);

			// Find the most distant point to the line between the two chosen extreme points.
			const Ray<FloatType> r(m_vertexData[selectedPoints.first], (m_vertexData[selectedPoints.second] - m_vertexData[selectedPoints.first]));
			maxD = m_epsilonSquared;
			size_t maxI = std::numeric_limits<size_t>::max();
			const size_t vCount = m_vertexData.size();
			for (size_t i = 0;i < vCount;i++) {
				const FloatType distToRay = mathutils::getSquaredDistanceBetweenPointAndRay(m_vertexData[i], r);
				if (distToRay > maxD) {
					maxD = distToRay;
					maxI = i;
				}
			}
			if (maxD == m_epsilonSquared) {
				// It appears that the point cloud belongs to a 1 dimensional subspace of R^3: convex hull has no volume => return a thin triangle
				// Pick any point other than selectedPoints.first and selectedPoints.second as the third point of the triangle
				auto it = std::find_if(m_vertexData.begin(), m_vertexData.end(), [&](const vec3& ve) {
					return ve != m_vertexData[selectedPoints.first] && ve != m_vertexData[selectedPoints.second];
				});
				const size_t thirdPoint = (it == m_vertexData.end()) ? selectedPoints.first : std::distance(m_vertexData.begin(), it);
				it = std::find_if(m_vertexData.begin(), m_vertexData.end(), [&](const vec3& ve) {
					return ve != m_vertexData[selectedPoints.first] && ve != m_vertexData[selectedPoints.second] && ve != m_vertexData[thirdPoint];
				});
				const size_t fourthPoint = (it == m_vertexData.end()) ? selectedPoints.first : std::distance(m_vertexData.begin(), it);
				return m_mesh.setup(selectedPoints.first, selectedPoints.second, thirdPoint, fourthPoint);
			}

			// These three points form the base triangle for our tetrahedron.
			assert(selectedPoints.first != maxI && selectedPoints.second != maxI);
			std::array<size_t, 3> baseTriangle{ selectedPoints.first, selectedPoints.second, maxI };
			const Vector3<FloatType> baseTriangleVertices[] = { m_vertexData[baseTriangle[0]], m_vertexData[baseTriangle[1]],  m_vertexData[baseTriangle[2]] };

			// Next step is to find the 4th vertex of the tetrahedron. We naturally choose the point farthest away from the triangle plane.
			maxD = m_epsilon;
			maxI = 0;
			const Vector3<FloatType> N = mathutils::getTriangleNormal(baseTriangleVertices[0], baseTriangleVertices[1], baseTriangleVertices[2]);
			Plane<FloatType> trianglePlane(N, baseTriangleVertices[0]);
			for (size_t i = 0;i < vCount;i++) {
				const FloatType d = std::abs(mathutils::getSignedDistanceToPlane(m_vertexData[i], trianglePlane));
				if (d > maxD) {
					maxD = d;
					maxI = i;
				}
			}
			if (maxD == m_epsilon) {
				// All the points seem to lie on a 2D subspace of R^3. How to handle this? Well, let's add one extra point to the point cloud so that the convex hull will have volume.
				m_planar = true;
				const vec3 N = mathutils::getTriangleNormal(baseTriangleVertices[1], baseTriangleVertices[2], baseTriangleVertices[0]);
				m_planarPointCloudTemp.clear();
				m_planarPointCloudTemp.insert(m_planarPointCloudTemp.begin(), m_vertexData.begin(), m_vertexData.end());
				const vec3 extraPoint = N + m_vertexData[0];
				m_planarPointCloudTemp.push_back(extraPoint);
				maxI = m_planarPointCloudTemp.size() - 1;
				m_vertexData = VertexDataSource<FloatType>(m_planarPointCloudTemp);
			}

			// Enforce CCW orientation (if user prefers clockwise orientation, swap two vertices in each triangle when final mesh is created)
			const Plane<FloatType> triPlane(N, baseTriangleVertices[0]);
			if (triPlane.isPointOnPositiveSide(m_vertexData[maxI])) {
				std::swap(baseTriangle[0], baseTriangle[1]);
			}

			// Create a tetrahedron half edge mesh and compute planes defined by each triangle
			m_mesh.setup(baseTriangle[0], baseTriangle[1], baseTriangle[2], maxI);
			for (auto& f : m_mesh.m_faces) {
				auto v = m_mesh.getVertexIndicesOfFace(f);
				const Vector3<FloatType>& va = m_vertexData[v[0]];
				const Vector3<FloatType>& vb = m_vertexData[v[1]];
				const Vector3<FloatType>& vc = m_vertexData[v[2]];
				const Vector3<FloatType> N = mathutils::getTriangleNormal(va, vb, vc);
				const Plane<FloatType> trianglePlane(N, va);
				f.m_P = trianglePlane;
			}

			// Finally we assign a face for each vertex outside the tetrahedron (vertices inside the tetrahedron have no role anymore)
			for (size_t i = 0; i < vCount; i++) {
				for (auto& face : m_mesh.m_faces) {
					if (addPointToFace(face, i)) {
						break;
					}
				}
			}
		}

		// Given a list of half edges, try to rearrange them so that they form a loop. Return true on success.
		bool reorderHorizonEdges(std::vector<size_t>& horizonEdges) {
			const size_t horizonEdgeCount = horizonEdges.size();
			for (size_t i = 0;i < horizonEdgeCount - 1;i++) {
				const size_t endVertex = m_mesh.m_halfEdges[horizonEdges[i]].m_endVertex;
				bool foundNext = false;
				for (size_t j = i + 1;j < horizonEdgeCount;j++) {
					const size_t beginVertex = m_mesh.m_halfEdges[m_mesh.m_halfEdges[horizonEdges[j]].m_opp].m_endVertex;
					if (beginVertex == endVertex) {
						std::swap(horizonEdges[i + 1], horizonEdges[j]);
						foundNext = true;
						break;
					}
				}
				if (!foundNext) {
					return false;
				}
			}
			assert(m_mesh.m_halfEdges[horizonEdges[horizonEdges.size() - 1]].m_endVertex == m_mesh.m_halfEdges[m_mesh.m_halfEdges[horizonEdges[0]].m_opp].m_endVertex);
			return true;
		}

		// Find indices of extreme values (max x, min x, max y, min y, max z, min z) for the given point cloud
		std::array<size_t,6> getExtremeValues() {
			std::array<size_t, 6> outIndices{ 0,0,0,0,0,0 };
			FloatType extremeVals[6] = { m_vertexData[0].x,m_vertexData[0].x,m_vertexData[0].y,m_vertexData[0].y,m_vertexData[0].z,m_vertexData[0].z };
			const size_t vCount = m_vertexData.size();
			for (size_t i = 1;i < vCount;i++) {
				const Vector3<FloatType>& pos = m_vertexData[i];
				if (pos.x > extremeVals[0]) {
					extremeVals[0] = pos.x;
					outIndices[0] = i;
				} else if (pos.x < extremeVals[1]) {
					extremeVals[1] = pos.x;
					outIndices[1] = i;
				}
				if (pos.y > extremeVals[2]) {
					extremeVals[2] = pos.y;
					outIndices[2] = i;
				} else if (pos.y < extremeVals[3]) {
					extremeVals[3] = pos.y;
					outIndices[3] = i;
				}
				if (pos.z > extremeVals[4]) {
					extremeVals[4] = pos.z;
					outIndices[4] = i;
				} else if (pos.z < extremeVals[5]) {
					extremeVals[5] = pos.z;
					outIndices[5] = i;
				}
			}
			return outIndices;
		}

		// Compute scale of the vertex data.
		FloatType getScale(const std::array<size_t,6>& extremeValues) {
			FloatType s = 0;
			for (size_t i = 0;i < 6;i++) {
				const FloatType* v = (const FloatType*)(&m_vertexData[extremeValues[i]]);
				v += i / 2;
				auto a = std::abs(*v);
				if (a > s) {
					s = a;
				}
			}
			return s;
		}

		// Each face contains a unique pointer to a vector of indices. However, many - often most - faces do not have any points on the positive
		// side of them especially at the the end of the iteration. When a face is removed from the mesh, its associated point vector, if such
		// exists, is moved to the index vector pool, and when we need to add new faces with points on the positive side to the mesh,
		// we reuse these vectors. This reduces the amount of std::vectors we have to deal with, and impact on performance is remarkable.
		Pool<std::vector<size_t>> m_indexVectorPool;
		inline std::unique_ptr<std::vector<size_t>> getIndexVectorFromPool();
		inline void reclaimToIndexVectorPool(std::unique_ptr<std::vector<size_t>>& ptr);

		// Associates a point with a face if the point resides on the positive side of the plane. Returns true if the points was on the positive side.
		inline bool addPointToFace(typename MeshBuilder<FloatType>::Face& f, size_t pointIndex);

		// This will update m_mesh from which we create the ConvexHull object that getConvexHull function returns
		void createConvexHalfEdgeMesh() {
			m_visibleFaces.clear();
			m_horizonEdges.clear();
			m_possiblyVisibleFaces.clear();

			// Compute base tetrahedron
			setupInitialTetrahedron();
			assert(m_mesh.m_faces.size() == 4);

			// Init face stack with those faces that have points assigned to them
			m_faceList.clear();
			for (size_t i = 0;i < 4;i++) {
				auto& f = m_mesh.m_faces[i];
				if (f.m_pointsOnPositiveSide && f.m_pointsOnPositiveSide->size() > 0) {
					m_faceList.push_back(i);
					f.m_inFaceStack = 1;
				}
			}

			// Process faces until the face list is empty.
			size_t iter = 0;
			while (!m_faceList.empty()) {
				iter++;
				if (iter == std::numeric_limits<size_t>::max()) {
					// Visible face traversal marks visited faces with iteration counter (to mark that the face has been visited on this iteration) and the max value represents unvisited faces. At this point we have to reset iteration counter. This shouldn't be an
					// issue on 64 bit machines.
					iter = 0;
				}

				const size_t topFaceIndex = m_faceList.front();
				m_faceList.pop_front();

				auto& tf = m_mesh.m_faces[topFaceIndex];
				tf.m_inFaceStack = 0;

				assert(!tf.m_pointsOnPositiveSide || tf.m_pointsOnPositiveSide->size() > 0);
				if (!tf.m_pointsOnPositiveSide || tf.isDisabled()) {
					continue;
				}

				// Pick the most distant point to this triangle plane as the point to which we extrude
				const vec3& activePoint = m_vertexData[tf.m_mostDistantPoint];
				const size_t activePointIndex = tf.m_mostDistantPoint;

				// Find out the faces that have our active point on their positive side (these are the "visible faces"). The face on top of the stack of course is one of them. At the same time, we create a list of horizon edges.
				m_horizonEdges.clear();
				m_possiblyVisibleFaces.clear();
				m_visibleFaces.clear();
				m_possiblyVisibleFaces.emplace_back(topFaceIndex, std::numeric_limits<size_t>::max());
				while (m_possiblyVisibleFaces.size()) {
					const auto faceData = m_possiblyVisibleFaces.back();
					m_possiblyVisibleFaces.pop_back();
					auto& pvf = m_mesh.m_faces[faceData.m_faceIndex];
					assert(!pvf.isDisabled());

					if (pvf.m_visibilityCheckedOnIteration == iter) {
						if (pvf.m_isVisibleFaceOnCurrentIteration) {
							continue;
						}
					} else {
						const Plane<FloatType>& P = pvf.m_P;
						pvf.m_visibilityCheckedOnIteration = iter;
						const FloatType d = P.m_N.dotProduct(activePoint) + P.m_D;
						if (d > 0) {
							pvf.m_isVisibleFaceOnCurrentIteration = 1;
							pvf.m_horizonEdgesOnCurrentIteration = 0;
							m_visibleFaces.push_back(faceData.m_faceIndex);
							for (auto heIndex : m_mesh.getHalfEdgeIndicesOfFace(pvf)) {
								if (m_mesh.m_halfEdges[heIndex].m_opp != faceData.m_enteredFromHalfEdge) {
									m_possiblyVisibleFaces.emplace_back(m_mesh.m_halfEdges[m_mesh.m_halfEdges[heIndex].m_opp].m_face, heIndex);
								}
							}
							continue;
						}
						assert(faceData.m_faceIndex != topFaceIndex);
					}

					// The face is not visible. Therefore, the halfedge we came from is part of the horizon edge.
					pvf.m_isVisibleFaceOnCurrentIteration = 0;
					m_horizonEdges.push_back(faceData.m_enteredFromHalfEdge);
					// Store which half edge is the horizon edge. The other half edges of the face will not be part of the final mesh so their data slots can by recycled.
					const auto halfEdges = m_mesh.getHalfEdgeIndicesOfFace(m_mesh.m_faces[m_mesh.m_halfEdges[faceData.m_enteredFromHalfEdge].m_face]);
					const std::int8_t ind = (halfEdges[0] == faceData.m_enteredFromHalfEdge) ? 0 : (halfEdges[1] == faceData.m_enteredFromHalfEdge ? 1 : 2);
					m_mesh.m_faces[m_mesh.m_halfEdges[faceData.m_enteredFromHalfEdge].m_face].m_horizonEdgesOnCurrentIteration |= (1 << ind);
				}
				const size_t horizonEdgeCount = m_horizonEdges.size();

				// Order horizon edges so that they form a loop. This may fail due to numerical instability in which case we give up trying to solve horizon edge for this point and accept a minor degeneration in the convex hull.
				if (!reorderHorizonEdges(m_horizonEdges)) {
					m_diagnostics.m_failedHorizonEdges++;
					std::cerr << "Failed to solve horizon edge." << std::endl;
					auto it = std::find(tf.m_pointsOnPositiveSide->begin(), tf.m_pointsOnPositiveSide->end(), activePointIndex);
					tf.m_pointsOnPositiveSide->erase(it);
					if (tf.m_pointsOnPositiveSide->size() == 0) {
						reclaimToIndexVectorPool(tf.m_pointsOnPositiveSide);
					}
					continue;
				}

				// Except for the horizon edges, all half edges of the visible faces can be marked as disabled. Their data slots will be reused.
				// The faces will be disabled as well, but we need to remember the points that were on the positive side of them - therefore
				// we save pointers to them.
				m_newFaceIndices.clear();
				m_newHalfEdgeIndices.clear();
				m_disabledFacePointVectors.clear();
				size_t disableCounter = 0;
				for (auto faceIndex : m_visibleFaces) {
					auto& disabledFace = m_mesh.m_faces[faceIndex];
					auto halfEdges = m_mesh.getHalfEdgeIndicesOfFace(disabledFace);
					for (size_t j = 0;j < 3;j++) {
						if ((disabledFace.m_horizonEdgesOnCurrentIteration & (1 << j)) == 0) {
							if (disableCounter < horizonEdgeCount * 2) {
								// Use on this iteration
								m_newHalfEdgeIndices.push_back(halfEdges[j]);
								disableCounter++;
							} else {
								// Mark for reusal on later iteration step
								m_mesh.disableHalfEdge(halfEdges[j]);
							}
						}
					}
					// Disable the face, but retain pointer to the points that were on the positive side of it. We need to assign those points
					// to the new faces we create shortly.
					auto t = std::move(m_mesh.disableFace(faceIndex));
					if (t) {
						assert(t->size()); // Because we should not assign point vectors to faces unless needed...
						m_disabledFacePointVectors.push_back(std::move(t));
					}
				}
				if (disableCounter < horizonEdgeCount * 2) {
					const size_t newHalfEdgesNeeded = horizonEdgeCount * 2 - disableCounter;
					for (size_t i = 0;i < newHalfEdgesNeeded;i++) {
						m_newHalfEdgeIndices.push_back(m_mesh.addHalfEdge());
					}
				}

				// Create new faces using the edgeloop
				for (size_t i = 0; i < horizonEdgeCount; i++) {
					const size_t AB = m_horizonEdges[i];

					auto horizonEdgeVertexIndices = m_mesh.getVertexIndicesOfHalfEdge(m_mesh.m_halfEdges[AB]);
					size_t A, B, C;
					A = horizonEdgeVertexIndices[0];
					B = horizonEdgeVertexIndices[1];
					C = activePointIndex;

					const size_t newFaceIndex = m_mesh.addFace();
					m_newFaceIndices.push_back(newFaceIndex);

					const size_t CA = m_newHalfEdgeIndices[2 * i + 0];
					const size_t BC = m_newHalfEdgeIndices[2 * i + 1];

					m_mesh.m_halfEdges[AB].m_next = BC;
					m_mesh.m_halfEdges[BC].m_next = CA;
					m_mesh.m_halfEdges[CA].m_next = AB;

					m_mesh.m_halfEdges[BC].m_face = newFaceIndex;
					m_mesh.m_halfEdges[CA].m_face = newFaceIndex;
					m_mesh.m_halfEdges[AB].m_face = newFaceIndex;

					m_mesh.m_halfEdges[CA].m_endVertex = A;
					m_mesh.m_halfEdges[BC].m_endVertex = C;

					auto& newFace = m_mesh.m_faces[newFaceIndex];

					const Vector3<FloatType> planeNormal = mathutils::getTriangleNormal(m_vertexData[A], m_vertexData[B], activePoint);
					newFace.m_P = Plane<FloatType>(planeNormal, activePoint);
					newFace.m_he = AB;

					m_mesh.m_halfEdges[CA].m_opp = m_newHalfEdgeIndices[i > 0 ? i * 2 - 1 : 2 * horizonEdgeCount - 1];
					m_mesh.m_halfEdges[BC].m_opp = m_newHalfEdgeIndices[((i + 1) * 2) % (horizonEdgeCount * 2)];
				}

				// Assign points that were on the positive side of the disabled faces to the new faces.
				for (auto& disabledPoints : m_disabledFacePointVectors) {
					assert(disabledPoints);
					for (const auto& point : *(disabledPoints)) {
						if (point == activePointIndex) {
							continue;
						}
						for (size_t j = 0;j < horizonEdgeCount;j++) {
							if (addPointToFace(m_mesh.m_faces[m_newFaceIndices[j]], point)) {
								break;
							}
						}
					}
					// The points are no longer needed: we can move them to the vector pool for reuse.
					reclaimToIndexVectorPool(disabledPoints);
				}

				// Increase face stack size if needed
				for (const auto newFaceIndex : m_newFaceIndices) {
					auto& newFace = m_mesh.m_faces[newFaceIndex];
					if (newFace.m_pointsOnPositiveSide) {
						assert(newFace.m_pointsOnPositiveSide->size() > 0);
						if (!newFace.m_inFaceStack) {
							m_faceList.push_back(newFaceIndex);
							newFace.m_inFaceStack = 1;
						}
					}
				}
			}

			// Cleanup
			m_indexVectorPool.clear();
		}

		// Constructs the convex hull into a MeshBuilder object which can be converted to a ConvexHull or Mesh object
		void buildMesh(const VertexDataSource<FloatType>& pointCloud, bool CCW, bool useOriginalIndices, FloatType eps) {
			if (pointCloud.size() == 0) {
				m_mesh = MeshBuilder<FloatType>();
				return;
			}
			m_vertexData = pointCloud;

			// Very first: find extreme values and use them to compute the scale of the point cloud.
			m_extremeValues = getExtremeValues();
			m_scale = getScale(m_extremeValues);

			// Epsilon we use depends on the scale
			m_epsilon = eps * m_scale;
			m_epsilonSquared = m_epsilon * m_epsilon;

			// Reset diagnostics
			m_diagnostics = DiagnosticsData();

			m_planar = false; // The planar case happens when all the points appear to lie on a two dimensional subspace of R^3.
			createConvexHalfEdgeMesh();
			if (m_planar) {
				const size_t extraPointIndex = m_planarPointCloudTemp.size() - 1;
				for (auto& he : m_mesh.m_halfEdges) {
					if (he.m_endVertex == extraPointIndex) {
						he.m_endVertex = 0;
					}
				}
				m_vertexData = pointCloud;
				m_planarPointCloudTemp.clear();
			}
		}

		// The public getConvexHull functions will setup a VertexDataSource object and call this
		ConvexHull<FloatType> getConvexHull(const VertexDataSource<FloatType>& pointCloud, bool CCW, bool useOriginalIndices, FloatType eps)
		{
			buildMesh(pointCloud, CCW, useOriginalIndices, epsilon);
			return ConvexHull<T>(m_mesh, m_vertexData, CCW, useOriginalIndices);
		}

	public:
		// Computes convex hull for a given point cloud.
		// Params:
		//   pointCloud: a vector of of 3D points
		//   CCW: whether the output mesh triangles should have CCW orientation
		//   useOriginalIndices: should the output mesh use same vertex indices as the original point cloud. If this is false,
		//      then we generate a new vertex buffer which contains only the vertices that are part of the convex hull.
		//   eps: minimum distance to a plane to consider a point being on positive of it (for a point cloud with scale 1)
		ConvexHull<FloatType> getConvexHull(const std::vector<Vector3<FloatType>>& pointCloud,
			bool CCW,
			bool useOriginalIndices,
			FloatType eps = defaultEps<FloatType>())
		{
			VertexDataSource<T> vertexDataSource(pointCloud);
			return getConvexHull(vertexDataSource, CCW, useOriginalIndices, eps);
		}

		// Computes convex hull for a given point cloud.
		// Params:
		//   vertexData: pointer to the first 3D point of the point cloud
		//   vertexCount: number of vertices in the point cloud
		//   CCW: whether the output mesh triangles should have CCW orientation
		//   useOriginalIndices: should the output mesh use same vertex indices as the original point cloud. If this is false,
		//      then we generate a new vertex buffer which contains only the vertices that are part of the convex hull.
		//   eps: minimum distance to a plane to consider a point being on positive side of it (for a point cloud with scale 1)
		ConvexHull<FloatType> getConvexHull(const Vector3<FloatType>* vertexData,
			size_t vertexCount,
			bool CCW,
			bool useOriginalIndices,
			FloatType eps = defaultEps<FloatType>())
		{
			VertexDataSource<T> vertexDataSource(vertexData, vertexCount);
			return getConvexHull(vertexDataSource, CCW, useOriginalIndices, eps);
		}

		// Computes convex hull for a given point cloud. This function assumes that the vertex data resides in memory
		// in the following format: x_0,y_0,z_0,x_1,y_1,z_1,...
		// Params:
		//   vertexData: pointer to the X component of the first point of the point cloud.
		//   vertexCount: number of vertices in the point cloud
		//   CCW: whether the output mesh triangles should have CCW orientation
		//   useOriginalIndices: should the output mesh use same vertex indices as the original point cloud. If this is false,
		//      then we generate a new vertex buffer which contains only the vertices that are part of the convex hull.
		//   eps: minimum distance to a plane to consider a point being on positive side of it (for a point cloud with scale 1)
		ConvexHull<FloatType> getConvexHull(const FloatType* vertexData,
			size_t vertexCount,
			bool CCW,
			bool useOriginalIndices,
			FloatType eps = defaultEps<FloatType>())
		{
			VertexDataSource<T> vertexDataSource((const vec3*)vertexData, vertexCount);
			return getConvexHull(vertexDataSource, CCW, useOriginalIndices, eps);
		}

		// Computes convex hull for a given point cloud. This function assumes that the vertex data resides in memory
		// in the following format: x_0,y_0,z_0,x_1,y_1,z_1,...
		// Params:
		//   vertexData: pointer to the X component of the first point of the point cloud.
		//   vertexCount: number of vertices in the point cloud
		//   CCW: whether the output mesh triangles should have CCW orientation
		//   eps: minimum distance to a plane to consider a point being on positive side of it (for a point cloud with scale 1)
		// Returns:
		//   Convex hull of the point cloud as a mesh object with half edge structure.
		HalfEdgeMesh<FloatType, size_t> getConvexHullAsMesh(const FloatType* vertexData,
															size_t vertexCount,
															bool CCW,
															FloatType eps = defaultEps<FloatType>()) {
			VertexDataSource<FloatType> vertexDataSource((const vec3*)vertexData, vertexCount);
			buildMesh(vertexDataSource, CCW, false, eps);
			return HalfEdgeMesh<FloatType, size_t>(m_mesh, m_vertexData);
		}

		void getConvexHullAsMesh2()
		{

		}

		// Get diagnostics about last generated convex hull
		const DiagnosticsData& getDiagnostics() {
			return m_diagnostics;
		}
	};

	/*
	 * Inline function definitions
	 */

	template<typename T>
	std::unique_ptr<std::vector<size_t>> QuickHull<T>::getIndexVectorFromPool() {
		auto r = std::move(m_indexVectorPool.get());
		r->clear();
		return r;
	}

	template<typename T>
	void QuickHull<T>::reclaimToIndexVectorPool(std::unique_ptr<std::vector<size_t>>& ptr) {
		const size_t oldSize = ptr->size();
		if ((oldSize+1)*128 < ptr->capacity()) {
			// Reduce memory usage! Huge vectors are needed at the beginning of iteration when faces have many points on their positive side. Later on, smaller vectors will suffice.
			ptr.reset(nullptr);
			return;
		}
		m_indexVectorPool.reclaim(ptr);
	}

	template<typename T>
	bool QuickHull<T>::addPointToFace(typename MeshBuilder<T>::Face& f, size_t pointIndex) {
		const T D = mathutils::getSignedDistanceToPlane(m_vertexData[ pointIndex ],f.m_P);
		if (D>0 && D*D > m_epsilonSquared*f.m_P.m_sqrNLength) {
			if (!f.m_pointsOnPositiveSide) {
				f.m_pointsOnPositiveSide = std::move(getIndexVectorFromPool());
			}
			f.m_pointsOnPositiveSide->push_back( pointIndex );
			if (D > f.m_mostDistantPointDist) {
				f.m_mostDistantPointDist = D;
				f.m_mostDistantPoint = pointIndex;
			}
			return true;
		}
		return false;
	}

}


#endif /* QUICKHULL_HPP_ */
