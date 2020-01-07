#include "../common/Common.hh"
#include "ShortestPath.hh"

#include <OpenFlipper/libs_required/ACG/Utils/HaltonColors.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

namespace betri
{

class VoronoiRemesh
{
public:

	using VH = ShortestPath::VH;
	using EH = ShortestPath::EH;
	using HH = ShortestPath::HH;
	using FH = ShortestPath::FH;

	using P = BezierTMesh::Point;

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

	VoronoiRemesh(BezierTMesh &mesh, BezierTMesh &ctrl) :
		m_mesh(mesh),
		m_ctrl(ctrl),
		m_useColors(true),
		m_overwrite(true),
		m_debugCancel(false),
		m_interpolate(false),
		m_vertexIdx(0u),
		m_minPartition(10u),
		m_colors(),
		m_seeds(),
		m_ctrlVerts()
	{
		m_minPartition = m_minPartition == 0 ? std::floor(0.01 * m_mesh.n_faces()) : m_minPartition;
		prepare();
	}

	~VoronoiRemesh()
	{
		cleanup();
	}

	// performs all steps
	bool remesh();

	// partitions mesh into voronoi regions
	bool partition(const bool stepwise, bool &done);

	// performs dualizing
	bool dualize(bool steps);
	// performs dualizing stepwise
	bool dualize(bool &done, bool steps);

	// parametrize and fit to surface
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

	void minPartition(size_t part) { m_minPartition = part; }
	size_t minPartition() const { return m_minPartition; }

	void weights(int which) { m_paramWeights = which; }
	int weights() const { return m_paramWeights; }

private:

	bool calcPartition(const bool stepwise, bool &done);

	void prepare();
	void cleanup();

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

	TriToVertex& ttv(FH fh)
	{
		return m_ctrl.property(m_ttv, fh);
	}
	VertexToTri& vtt(VH vh)
	{
		return m_mesh.property(m_vtt, vh);
	}

	ID& crossed(EH eh)
	{
		return m_mesh.property(m_crossed, eh);
	}
	ID& crossed(HH he)
	{
		return crossed(m_mesh.edge_handle(he));
	}

	ID crossed(const EH eh) const
	{
		return m_mesh.property(m_crossed, eh);
	}
	ID crossed(const HH he) const
	{
		return crossed(m_mesh.edge_handle(he));
	}

	bool isCrossed(const EH eh) const
	{
		return eh.is_valid() && crossed(eh) >= 0;
	}
	bool isCrossed(const HH he) const
	{
		return he.is_valid() && isCrossed(m_mesh.edge_handle(he));
	}
	bool isCrossed(const VH vh) const
	{
		for (auto e_it = m_mesh.cve_begin(vh); e_it != m_mesh.cve_end(vh); ++e_it) {
			if (isCrossed(*e_it)) return true;
		}
		return false;
	}

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

	void resetPath(FaceDijkstra &q, const FH face);

	using VQElem = std::pair<Scalar, VH>;
	using VertexDijkstra = std::set<VQElem>;

	void vertexDijkstra(const ID id0=-1, const ID id1=-1);

	EH commonEdge(const FH f0, const FH f1)
	{
		for (auto ff = m_mesh.cfh_begin(f0); ff != m_mesh.cfh_end(f0); ++ff) {
			if (m_mesh.opposite_face_handle(*ff) == f1) {
				return m_mesh.edge_handle(*ff);
			}
		}
		return EH();
	}

	void grow(FaceDijkstra &q, FH face, const FH predFace=FH(), Scalar distance=0.0)
	{
		id(face) = !predFace.is_valid() ? m_seeds.size() - 1 : id(predFace);
		setColor(face, m_colors[id(face)]);
		pred(face) = predFace;

		assert(!predFace.is_valid() || face != pred(pred(face)));

		auto pair = FQElem(dist(face), face);
		auto it = q.find(pair);
		if (it != q.end()) q.erase(it);

		dist(face) = distance;
		pair.first = distance;
		q.insert(pair);
	};

	void growTiles(FaceDijkstra &q);

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

	bool partitionIsValid();

	void findRegionCuts(std::vector<std::vector<std::vector<FH>>> &cuts);

	void reduceCuts(FaceDijkstra &q);

	void reduceAdjRegions(FaceDijkstra &q);

	bool isSeed(const FH f) const
	{
		if (id(f) >= 0) {
			return id(f) < m_seeds.size() && m_seeds[id(f)] == f;
		} else {
			return std::find(m_seeds.begin(), m_seeds.end(), f) != m_seeds.end();
		}
	}

	void addSeed(FaceDijkstra &q, const FH f)
	{
		assert(!isSeed(f));
		m_seeds.push_back(f);
		if (m_useColors) {
			m_colors.push_back(m_colGen.generateNextColor());
		}
		grow(q, f);

		// reduce alpha so seed faces are visible
		Color c = m_colors[id(f)];
		c[3] = 0.5f;
		setColor(f, c);
	}

	static void copyMesh(BezierTMesh &src, BezierTMesh &dest);

	void preventiveEdgeSplits();

	void assignInnerVertices();

	void splitClosedPaths(std::set<ID> only);

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

	void shortestPath(
		const VH v,
		const ID id_1,
		const ID id_2,
		const FH ctrlFace,
		ShortestPath &path
	);

	bool isSeedVertex(const VH vh) const
	{
		assert(id(vh) >= 0 || id(vh) == BORDER_ID);
		if (id(vh) == BORDER_ID) return false;

		return m_seedVerts[id(vh)] == vh;
	}

	FH findDelaunayFace(ID r0, ID r1, ID r2) {
		for (FH f : m_ctrl.faces()) {
			if (ttv(f).isRegion(r0, r1, r2)) {
				return f;
			}
		}
		return FH();
	}

	bool adjToRegion(const VertexHandle vh, const ID id0) const
	{
		for (auto f_it = m_mesh.cvf_begin(vh), f_e = m_mesh.cvf_end(vh); f_it != f_e; ++f_it) {
			if (id(*f_it) == id0) return true;
		}
		return false;
	}

	bool adjToSeedFace(const FaceHandle fh) const
	{
		for (auto v_it = m_mesh.cfv_begin(fh), v_e = m_mesh.cfv_end(fh); v_it != v_e; ++v_it) {
			//if (countAdjRegions(*v_it) > 2) return true;
			for (auto f_it = m_mesh.cvf_begin(*v_it), f_e = m_mesh.cvf_end(*v_it); f_it != f_e; ++f_it) {
				if (isSeed(*f_it)) return true;
			}
		}
		/*for (auto f_it = m_mesh.cff_begin(fh), f_e = m_mesh.cff_end(fh); f_it != f_e; ++f_it) {
			if (isSeed(*f_it)) return true;
		}*/
		return false;
	}

	bool nextToSeed(const FaceHandle fh) const
	{
		for (auto f_it = m_mesh.cff_begin(fh), f_e = m_mesh.cff_end(fh); f_it != f_e; ++f_it) {
			if (isSeed(*f_it)) return true;
		}
		return false;
	}

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

	int countAdjRegions(const VH vh) const
	{
		std::set<ID> adj;
		for (auto f_it = m_mesh.cvf_begin(vh); f_it != m_mesh.cvf_end(vh); ++f_it) {
			if (id(*f_it) >= 0) adj.insert(id(*f_it));
		}
		return adj.size();
	}

	int countRegionFaces(const VH vh, const ID id0) const
	{
		int adj = 0;
		for (auto f_it = m_mesh.cvf_begin(vh); f_it != m_mesh.cvf_end(vh); ++f_it) {
			if (id(*f_it) == id0) adj++;
		}
		return adj;
	}

	bool onRegionBorder(const VH vh) const
	{
		return countAdjRegions(vh) > 1;
	}

	bool onRegionBorder(const FH fh) const
	{
		for (auto v_it = m_mesh.cfv_begin(fh), v_e = m_mesh.cfv_end(fh); v_it != v_e; ++v_it) {
			if (onRegionBorder(*v_it)) return true;
		}
		return false;
	}

	bool onRegionBorder(const VH vh, const ID id0, const ID id1) const
	{
		bool r0 = false, r1 = false;

		for (auto f_it = m_mesh.cvf_begin(vh); f_it != m_mesh.cvf_end(vh); ++f_it) {
			if (id(*f_it) == id0) r0 = true;
			else if (id(*f_it) == id1) r1 = true;
		}
		return r0 && r1;
	}

	bool addExtraSeed(FaceDijkstra &q);

	bool faceSP(FaceDijkstra &q, const bool stepwise);

	void ensureReachable(const ID id0);

	void vertexSP(VertexDijkstra &q);

	void findShortestPath(const VH vh, const ID id0);

	void setShortestPath(const VH vh);

	void setColor(VH vh, const Color &color) const
	{
		if (useColors()) {
			m_mesh.set_color(vh, color);
		}
	}

	void setColor(HH hh, const Color &color) const
	{
		if (useColors()) {
			m_mesh.set_color(hh, color);
		}
	}

	void setColor(EH eh, const Color &color) const
	{
		if (useColors()) {
			m_mesh.set_color(eh, color);
		}
	}

	void setColor(FH fh, const Color &color) const
	{
		if (useColors()) {
			m_mesh.set_color(fh, color);
		}
	}

	// -------------------------------------------------------------- //
	// member variables
	// -------------------------------------------------------------- //

	static constexpr ID BORDER_ID = -2;

	BezierTMesh &m_mesh, &m_ctrl;

	std::string m_errorMsg;

	bool m_useColors, m_overwrite, m_debugCancel, m_interpolate;
	size_t m_nvertices, m_nedges, m_vertexIdx, m_minPartition;
	// 0 = uniform, 1 = contangent
	int m_paramWeights;

	ACG::HaltonColors m_colGen;

	std::vector<Color> m_colors;
	std::vector<FH> m_seeds;
	std::vector<VH> m_ctrlVerts;
	std::vector<VH> m_seedVerts;

	FaceDijkstra m_q;

	// property handles
	OpenMesh::FPropHandleT<ID>			  m_region;
	OpenMesh::FPropHandleT<FH>			  m_pred;
	OpenMesh::FPropHandleT<Scalar>		  m_distance;
	// must be something other than bool because vector<bool> is handled uniquely in C++
	OpenMesh::EPropHandleT<ID>			  m_crossed;
	OpenMesh::VPropHandleT<ID>			  m_vid;
	OpenMesh::VPropHandleT<Scalar>		  m_vdist;
	OpenMesh::VPropHandleT<VH>			  m_vpred;
	OpenMesh::VPropHandleT<bool>		  m_vborder;

	OpenMesh::FPropHandleT<TriToVertex>   m_ttv;
	OpenMesh::VPropHandleT<VertexToTri>	  m_vtt;
};

}
