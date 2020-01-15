#include "../common/Common.hh"
#include "VoronoiFitting.hh"
#include "ShortestPath.hh"

#include <OpenFlipper/libs_required/ACG/Utils/HaltonColors.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

namespace betri
{

class VoronoiRemesh
{
public:

	// typedefs for even shorter code ...
	using VH = ShortestPath::VH;
	using EH = ShortestPath::EH;
	using HH = ShortestPath::HH;
	using FH = ShortestPath::FH;

	using P = BezierTMesh::Point;

	/**
	 * @brief Contains all property names
	 *
	 */
	struct Props
	{
		static constexpr char *REGION = "region";
		static constexpr char *PREDECESSOR = "pred";
		static constexpr char *DISTANCE = "dist";
		static constexpr char *CROSSED = "crossed";

		static constexpr char *VERTEXTOTRI = "vtt";
		static constexpr char *TRITOVERTEX = "ttv";
		static constexpr char *BORDER = "border";
	};

	/**
	 * @brief Constructs a new Voronoi Remesh object
	 *
	 * @param mesh mesh to perform meshing for
	 * @param ctrl control mesh (dual)
	 */
	VoronoiRemesh(BezierTMesh &mesh, BezierTMesh &ctrl) :
		m_mesh(mesh),
		m_ctrl(ctrl),
		m_useColors(true),
		m_overwrite(true),
		m_debugCancel(false),
		m_interpolate(false),
		m_vertexIdx(0u),
		m_minPartition(10u),
		m_fittingSamples(30u),
		m_fittingSolver(Fitting::Solver::adaptive),
		m_colors(),
		m_seeds(),
		m_ctrlVerts()
	{
		m_minPartition = m_minPartition == 0 ? std::floor(0.01 * m_mesh.n_faces()) : m_minPartition;
		prepare();
	}

	/**
	 * @brief Destroys the Voronoi Remesh object
	 */
	~VoronoiRemesh()
	{
		cleanup();
	}

	/**
	 * @brief Performs the complete algorithm.
	 *
	 * @return true if algorithm was successful
	 * @return false else
	 */
	bool remesh();

	/**
	 * @brief Partitions mesh into voronoi regions.
	 *
	 * @param stepwise perform only a single step
	 * @param done whether partition is finished
	 * @return true if partition was successful
	 * @return false else
	 */
	bool partition(const bool stepwise, bool &done);

	/**
	 * @brief Performs dualizing.
	 *
	 * @param steps only perform one step if true
	 * @return true if dualizing was successful
	 * @return false else
	 */
	bool dualize(bool steps);
	/**
	 * @brief Performs dualizing.
	 *
	 * @param done whether duallizing finished
	 * @param steps only perform one step if true
	 * @return true if dualizing was successful
	 * @return false else
	 */
	bool dualize(bool &done, bool steps);

	// parametrize and fit to surface
	/**
	 * @brief Parametrizes and fits all faces of the control mesh.
	 *
	 * If overwrite is enables, the control mesh ic copied to
	 * the source mesh.
	 *
	 * @return true if fitting was successful
	 * @return false else
	 */
	bool fitting();

	// -------------------------------------
	// settings
	// -------------------------------------

	void useColors(bool use) { m_useColors = use; }
	bool useColors() const { return m_useColors; }

	void interpolate(bool use) { m_interpolate = use; }
	bool interpolate() const { return m_interpolate; }

	void overwrite(bool use) { m_overwrite = use; }
	bool overwrite() const { return m_overwrite; }

	void splits(bool use) { m_splits = use; }
	bool splits() const { return m_splits; }

	void minPartition(size_t number) { m_minPartition = number; }
	size_t minPartition() const { return m_minPartition; }

	void fittingSamples(size_t number) { m_fittingSamples = number; }
	size_t fittingSamples() const { return m_fittingSamples; }

	void fittingSolver(Fitting::Solver solver) { m_fittingSolver = solver; }
	Fitting::Solver fittingSolver() const { return m_fittingSolver; }

	void weights(int which) { m_paramWeights = which; }
	int weights() const { return m_paramWeights; }

private:

	/**
	 * @brief Resets all values for repeated execution of the algorithm.
	 *
	 * This is necessary e.g. when partitioning should be done again.
	 *
	 */
	void reset();

	/**
	 * @brief Calculates the mesh partition using a face-based multi-source Dijkstra.
	 *
	 * @param stepwise only perform one step if true
	 * @param done whether the partition is finished
	 * @return true if successful
	 * @return false  else
	 */
	bool calcPartition(const bool stepwise, bool &done);

	void prepare();
	void cleanup();

	/**
	 * @brief Split edges longer than 4/3 of the mean edge length.
	 *
	 * This is useful is the mesh gemoetry is not that "nice" or if the
	 * mesh is rather small concerning complexity.
	 */
	void splitLongEdges();

	bool debugCancel() const
	{
		if (m_debugCancel) std::cerr << m_errorMsg << std::endl;
		return m_debugCancel;
	}
	void debugCancel(const char *msg)
	{
		m_debugCancel = true;
		m_errorMsg += msg;
		m_errorMsg += '\n';
	}
	void debugCancel(std::string msg)
	{
		m_debugCancel = true;
		m_errorMsg += msg;
		m_errorMsg += '\n';
	}

	ID& id(FH fh) { return m_mesh.property(m_region, fh); }
	ID id(const FH fh) const { return m_mesh.property(m_region, fh); }
	ID& id(VH vh) { return m_mesh.property(m_vid, vh); }
	ID id(const VH vh) const { return m_mesh.property(m_vid, vh); }

	FH& pred(FH fh) { return m_mesh.property(m_pred, fh); }
	FH pred(const FH fh) const { return m_mesh.property(m_pred, fh); }
	VH& pred(VH vh) { return m_mesh.property(m_vpred, vh); }
	VH pred(const VH vh) const { return m_mesh.property(m_vpred, vh); }

	Scalar& dist(FH fh) { return m_mesh.property(m_distance, fh); }
	Scalar& dist(VH vh) { return m_mesh.property(m_vdist, vh); }

	bool border(const VH vh) const { return m_mesh.property(m_vborder, vh); }
	void border(VH vh, bool val) { m_mesh.property(m_vborder, vh) = val; }

	TriToVertex& ttv(FH fh) { return m_ctrl.property(m_ttv, fh); }
	VertexToTri& vtt(VH vh) { return m_mesh.property(m_vtt, vh); }

	ID& crossed(EH eh) { return m_mesh.property(m_crossed, eh); }
	ID& crossed(HH he) { return crossed(m_mesh.edge_handle(he)); }

	ID crossed(const EH eh) const { return m_mesh.property(m_crossed, eh); }
	ID crossed(const HH he) const { return crossed(m_mesh.edge_handle(he)); }

	bool isCrossed(const EH eh) const { return eh.is_valid() && crossed(eh) >= 0; }
	bool isCrossed(const HH he) const { return he.is_valid() && isCrossed(m_mesh.edge_handle(he)); }
	bool isCrossed(const VH vh) const
	{
		for (auto e_it = m_mesh.cve_begin(vh); e_it != m_mesh.cve_end(vh); ++e_it) {
			if (isCrossed(*e_it)) return true;
		}
		return false;
	}

	/**
	 * @brief Returns whether the common edge of two faces was crossed
	 *
	 * @param fh face 1
	 * @param next adjacent face
	 * @return true if the common edge was crossed
	 * @return false else
	 */
	bool commonEdgeCrossed(const FH fh, const FH next) const
	{
		assert(m_mesh.adjToFace(fh, next));
		if (!pred(next).is_valid()) return false;

		for (auto h_it = m_mesh.cfh_begin(fh); h_it != m_mesh.cfh_end(fh); ++h_it) {
			if (m_mesh.opposite_face_handle(*h_it) == next) {
				return isCrossed(*h_it);
			}
		}
		return false;
	}

	// everythign related to the voronoi partition
	using FQElem = std::pair<Scalar, FH>;
	using FaceDijkstra = std::set<FQElem>;

	using VQElem = std::pair<Scalar, VH>;
	using VertexDijkstra = std::set<VQElem>;

	/**
	 * @brief Performs the vertex-based Dijkstra for the given regions.
	 * If the regions are set to -1, the shortest paths for all regions
	 * are calculated
	 *
	 * @param id0 region 1
	 * @param id1 region 2
	 */
	void vertexDijkstra(const ID id0=-1, const ID id1=-1);

	/**
	 * @brief Get the common edge of two faces.
	 *
	 * @param f0
	 * @param f1
	 * @return EH
	 */
	EH commonEdge(const FH f0, const FH f1)
	{
		for (auto ff = m_mesh.cfh_begin(f0); ff != m_mesh.cfh_end(f0); ++ff) {
			if (m_mesh.opposite_face_handle(*ff) == f1) {
				return m_mesh.edge_handle(*ff);
			}
		}
		return EH();
	}

	/**
	 * @brief Grow a region for the partition by adding face to it.
	 *
	 * @param q
	 * @param face
	 * @param predFace
	 * @param distance
	 */
	void grow(FaceDijkstra &q, FH face, const FH predFace=FH(), Scalar distance=0.0)
	{
		id(face) = !predFace.is_valid() ? m_seeds.size() - 1 : id(predFace);
		// must be enclosed by if because of the array access
		if (useColors()) {
			setColor(face, getRegionColor(face));
		}
		pred(face) = predFace;

		assert(!predFace.is_valid() || face != pred(pred(face)));

		auto pair = FQElem(dist(face), face);
		auto it = q.find(pair);
		if (it != q.end()) q.erase(it);

		dist(face) = distance;
		pair.first = distance;
		q.insert(pair);
	};

	/**
	 * @brief Grow all tiles/regions of the partition until all faces
	 * are inside one region.
	 *
	 * @param q
	 */
	void growTiles(FaceDijkstra &q);

	/**
	 * @brief Check whether adding face "f" retains disk topology
	 * for the region with id "tile"
	 *
	 * @param f
	 * @param cross
	 * @param tile
	 * @return true
	 * @return false
	 */
	bool homeomorphicDisk(const FH f, const HH cross, const ID tile) const
	{
		HH left = m_mesh.next_halfedge_handle(cross);
		HH right = m_mesh.next_halfedge_handle(left);

		ID fh0 = id(m_mesh.opposite_face_handle(left));
		ID fh1 = id(m_mesh.opposite_face_handle(right));

		return !(fh0 != tile && fh1 != tile &&
			adjToRegion(m_mesh.to_vertex_handle(left), tile)
		);
	}

	/**
	 * @brief Checks whether the partition is valid.
	 *
	 * @return true is valid
	 * @return false else
	 */
	bool partitionIsValid();

	/**
	 * @brief Finds all regions cuts.
	 *
	 * @param cuts data structure to store cuts
	 */
	void findRegionCuts(std::vector<std::vector<std::vector<FH>>> &cuts);

	/**
	 * @brief Add new seeds where two regions share too many cuts.
	 *
	 * @param q
	 */
	void reduceCuts(FaceDijkstra &q);

	/**
	 * @brief Adds seeds where a vertex has more than 3 adjacent regions.
	 *
	 * @param q
	 */
	void reduceAdjRegions(FaceDijkstra &q);

	/**
	 * @brief Returns whether the given face is a seed face.
	 *
	 * @param f
	 * @return true if f is a seed face
	 * @return false
	 */
	bool isSeed(const FH f) const
	{
		if (id(f) >= 0) {
			return id(f) < m_seeds.size() && m_seeds[id(f)] == f;
		} else {
			return std::find(m_seeds.begin(), m_seeds.end(), f) != m_seeds.end();
		}
	}

	/**
	 * @brief Adds a new seed face.
	 *
	 * @param q
	 * @param f
	 */
	void addSeed(FaceDijkstra &q, const FH f)
	{
		assert(!isSeed(f));
		m_seeds.push_back(f);
		if (useColors()) {
			m_colors.push_back(m_colGen.generateNextColor());
		}
		grow(q, f);

		if (useColors()) {
			// reduce alpha so seed faces are visible
			Color c = getRegionColor(f);
			c[3] = 0.5f;
			setColor(f, c);
		}
	}

	/**
	 * @brief Copies data from source to destination mesh.
	 *
	 * @param src
	 * @param dest
	 */
	static void copyMesh(BezierTMesh &src, BezierTMesh &dest);

	/**
	 * @brief Splits edges that are the only edge in the cut of two regions.
	 *
	 */
	void preventiveEdgeSplits();

	/**
	 * @brief Assigns the vertices inside the regions induced by the dualizing
	 * to their corresponding dual face.
	 *
	 */
	void assignInnerVertices();

	/**
	 * @brief Splits closed paths for the regions contained in the given set.
	 *
	 * @param only
	 */
	void splitClosedPaths(std::set<ID> only);

	/**
	 * @brief Previously used to fix predecessor relations (very messy). Does nothing.
	 *
	 * @param fh
	 * @param rewrite
	 */
	void fixPredecessor(const FH fh, const bool rewrite=false);

	template <class Container>
	void fixAllFaces(Container &q1, Container &q2)
	{
		//std::cerr << "FIX INNER FACES\n";
		for (FH face : q1) {
			if (!m_mesh.status(face).tagged()) fixPredecessor(face, true);
		}
		//std::cerr << "FIX OUTER FACES\n";
		for (FH face : q2) {
			if (!m_mesh.status(face).tagged()) fixPredecessor(face);
		}
		// reset tags
		for (FH face : q1) {
			m_mesh.status(face).set_tagged(false);
		}
		for (FH face : q2) {
			m_mesh.status(face).set_tagged(false);
		}
	}

	HalfedgeHandle fixCrossing(
		const FH f0,
		const FH f1,
		const VH prevNode=VH(),
		const VH notNode=VH()
	);

	/**
	 * @brief Find the shortest paths between tow regions using
	 * predecessors and stores them in the given ShortestPath object.
	 *
	 * @param v voronoi vertex that initiated this search
	 * @param id_1 region 1 id/index
	 * @param id_2 region 2 id/index
	 * @param ctrlFace corresponding dual face
	 * @param path ShortestPath object that is filled with path vertices
	 */
	void shortestPath(
		const VH v,
		const ID id_1,
		const ID id_2,
		const FH ctrlFace,
		ShortestPath &path
	);

	/**
	 * @brief Returns whether the given vertex is a seed.
	 *
	 * @param vh
	 * @return true if vh is a seed vertex
	 * @return false
	 */
	bool isSeedVertex(const VH vh) const
	{
		if (id(vh) < 0) return false;

		return m_seedVerts[id(vh)] == vh;
	}

	/**
	 * @brief Finds the dual face for a given combination of 3 region ids.
	 *
	 * @param r0 region 1 id/index
	 * @param r1 region 2 id/index
	 * @param r2 region 3 id/index
	 * @return FH dual face
	 */
	FH findDelaunayFace(ID r0, ID r1, ID r2) {
		for (FH f : m_ctrl.faces()) {
			if (ttv(f).isRegion(r0, r1, r2)) {
				return f;
			}
		}
		return FH();
	}

	/**
	 * @brief Returns whether a vertex is adjacent to a specfic region (face)
	 *
	 * @param vh
	 * @param id0
	 * @return true
	 * @return false
	 */
	bool adjToRegion(const VertexHandle vh, const ID id0) const
	{
		for (auto f_it = m_mesh.cvf_begin(vh), f_e = m_mesh.cvf_end(vh); f_it != f_e; ++f_it) {
			if (id(*f_it) == id0) return true;
		}
		return false;
	}

	/**
	 * @brief Return wether the given face is adjacent to another seed face.
	 *
	 * @param fh
	 * @return true
	 * @return false
	 */
	bool adjToSeedFace(const FaceHandle fh) const
	{
		for (auto v_it = m_mesh.cfv_begin(fh), v_e = m_mesh.cfv_end(fh); v_it != v_e; ++v_it) {
			for (auto f_it = m_mesh.cvf_begin(*v_it), f_e = m_mesh.cvf_end(*v_it); f_it != f_e; ++f_it) {
				if (isSeed(*f_it)) return true;
			}
		}
		return false;
	}

	/**
	 * @brief Returns whether the given face is directly next to a seed face.
	 *
	 * @param fh
	 * @return true
	 * @return false
	 */
	bool nextToSeed(const FaceHandle fh) const
	{
		for (auto f_it = m_mesh.cff_begin(fh), f_e = m_mesh.cff_end(fh); f_it != f_e; ++f_it) {
			if (isSeed(*f_it)) return true;
		}
		return false;
	}

	/**
	 * @brief Finds the potential predecessor with the minimum distance.
	 *
	 * @param vh
	 * @param noBorder
	 * @return VH
	 */
	VH minPredecessor(const VertexHandle vh, bool noBorder=false)
	{
		Scalar minDist = std::numeric_limits<Scalar>::max();
		VH minPred;
		for (auto v_it = m_mesh.cvv_begin(vh); v_it != m_mesh.cvv_end(vh); ++v_it) {
			if (id(*v_it) >= 0 && dist(*v_it) < minDist && adjToRegion(vh, id(*v_it)) &&
				(noBorder || !vtt(*v_it).isBorder())) {
				minPred = *v_it;
				minDist = dist(*v_it);
			}

		}
		return minPred;
	}

	/**
	 * @brief Counts and return the number of distinct adjacent regions of
	 * the given vertex.
	 *
	 * @param vh
	 * @return int
	 */
	int countAdjRegions(const VH vh) const
	{
		std::set<ID> adj;
		for (auto f_it = m_mesh.cvf_begin(vh); f_it != m_mesh.cvf_end(vh); ++f_it) {
			if (id(*f_it) >= 0) adj.insert(id(*f_it));
		}
		return adj.size();
	}

	/**
	 * @brief Counts the number of adjacent faces that are inside the
	 * region with the given id.
	 *
	 * @param vh
	 * @param id0
	 * @return int
	 */
	int countRegionFaces(const VH vh, const ID id0) const
	{
		int adj = 0;
		for (auto f_it = m_mesh.cvf_begin(vh); f_it != m_mesh.cvf_end(vh); ++f_it) {
			if (id(*f_it) == id0) adj++;
		}
		return adj;
	}

	/**
	 * @brief Returns whether the given vertex lies in the border of regions.
	 *
	 * @param vh
	 * @return true
	 * @return false
	 */
	bool onRegionBorder(const VH vh) const
	{
		return countAdjRegions(vh) > 1;
	}

	/**
	 * @brief Returns whether the given face is part of the region border.
	 *
	 * @param fh
	 * @return true
	 * @return false
	 */
	bool onRegionBorder(const FH fh) const
	{
		for (auto v_it = m_mesh.cfv_begin(fh), v_e = m_mesh.cfv_end(fh); v_it != v_e; ++v_it) {
			if (onRegionBorder(*v_it)) return true;
		}
		return false;
	}

	/**
	 * @brief Returns whether the given vertex is on the border
	 * between the two given regions.
	 *
	 * @param vh
	 * @param id0
	 * @param id1
	 * @return true
	 * @return false
	 */
	bool onRegionBorder(const VH vh, const ID id0, const ID id1) const
	{
		bool r0 = false, r1 = false;

		for (auto f_it = m_mesh.cvf_begin(vh); f_it != m_mesh.cvf_end(vh); ++f_it) {
			if (id(*f_it) == id0) r0 = true;
			else if (id(*f_it) == id1) r1 = true;
		}
		return r0 && r1;
	}

	/**
	 * @brief Adds an additional seed face. A seed with a large distance is
	 * chosen as a candidate, as long as it is not in the neighorhood
	 * of another seed face.
	 *
	 * @param q
	 * @return true if a seed was added
	 * @return false else
	 */
	bool addExtraSeed(FaceDijkstra &q);

	/**
	 * @brief Calculates shortest paths over faces.
	 *
	 * @param q
	 * @param stepwise
	 * @return true
	 * @return false
	 */
	bool faceSP(FaceDijkstra &q, const bool stepwise);

	/**
	 * @brief Calculates shortest paths over vertices.
	 *
	 * @param q
	 */
	void vertexSP(VertexDijkstra &q);

	void findShortestPath(const VH vh, const ID id0);

	template <typename T>
	void setColor(T handle, const Color &color) const
	{
		if (useColors()) {
			m_mesh.set_color(handle, color);
		}
	}

	template <typename T>
	Color getRegionColor(T handle) const
	{
		ID index = id(handle);
		if (useColors() && index >= 0 && index < m_colors.size()) {
			return m_colors[index];
		}
		return Color();
	}

	// -------------------------------------------------------------- //
	// member variables
	// -------------------------------------------------------------- //

	//! reference to target mesh
	BezierTMesh &m_mesh;
	//! reference to control mesh
	BezierTMesh &m_ctrl;

	//! accumulated error messages
	std::string m_errorMsg;

	//! whether to use colors
	bool m_useColors;
	//! whether to copy from control to target mesh
	bool m_overwrite;
	//! whether to cancel the algorithm
	bool m_debugCancel;
	//! whether to sue interpolation instead of fixing control points for fitting
	bool m_interpolate;
	//! whether to split long edges at the start
	bool m_splits;

	//! nmber of vertices
	size_t m_nvertices;
	//! number of edges
	size_t m_nedges;
	//! index if the current vertex (used for stepwise dualizing)
	size_t m_vertexIdx;
	//! minimum number for regions
	size_t m_minPartition;
	//! maximum number of fitting samples to use
	size_t m_fittingSamples;
	//! which parametrization weights to use: 0 = uniform, 1 = contangent
	int m_paramWeights;
	//! which fitting solver to use
	Fitting::Solver m_fittingSolver;

	//! color generator for paritioning
	ACG::HaltonColors m_colGen;

	//! region colors
	std::vector<Color> m_colors;
	//! seed faces
	std::vector<FH> m_seeds;
	//! vertices (one for each seed face) in the control mesh
	std::vector<VH> m_ctrlVerts;
	//! seed vertices
	std::vector<VH> m_seedVerts;

	//! q for face-based Dijkstra (needed for stepwise execution)
	FaceDijkstra m_q;

	//! property handle for face region ids
	OpenMesh::FPropHandleT<ID>			  m_region;
	//! property handle for face predecessors
	OpenMesh::FPropHandleT<FH>			  m_pred;
	//! property handle for face distance to seed
	OpenMesh::FPropHandleT<Scalar>		  m_distance;

	// must be something other than bool because vector<bool> is handled uniquely in C++
	//! edgesfor which regions an edge was crossed
	OpenMesh::EPropHandleT<ID>			  m_crossed;
	//! property handle for vertex region ids
	OpenMesh::VPropHandleT<ID>			  m_vid;
	//! property handle for vertex distances to seed
	OpenMesh::VPropHandleT<Scalar>		  m_vdist;
	//! property handle for vertex predecessors
	OpenMesh::VPropHandleT<VH>			  m_vpred;
	//! property handle to determine if vertex is on a border (redundant/unused?)
	OpenMesh::VPropHandleT<bool>		  m_vborder;

	//! property handle for parametrization map (face -> vertices)
	OpenMesh::FPropHandleT<TriToVertex>   m_ttv;
	//! property handle for parametrization map (vertex -> face)
	OpenMesh::VPropHandleT<VertexToTri>	  m_vtt;
};

}
