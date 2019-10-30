#include "../common/Common.hh"
#include "ShortestPath.hh"

#include <OpenFlipper/libs_required/ACG/Utils/HaltonColors.hh>

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

	VoronoiRemesh(BezierTMesh &mesh, BezierTMesh &ctrl, bool colors = true, bool copy = false) :
		m_mesh(mesh),
		m_ctrl(ctrl),
		m_useColors(colors),
		m_copy(copy),
		m_debugCancel(false),
		m_useBaseMesh(false),
		m_vertexIdx(0u),
		m_colors(),
		m_seeds(),
		m_ctrlVerts()
	{
		prepare();
	}

	~VoronoiRemesh()
	{
		cleanup();
	}

	// performs all steps
	void remesh();

	// partitions mesh into voronoi regions
	void partition();
	// use orignal mesh and initialize all needed data structures
	void prepareFromBaseMesh();

	// performs "dualizing" by finding paths on the mesh, can be performed stepwise
	bool dualize(bool steps=false);
	// initialize shortest paths if original mesh is used as base
	void makeShortestPaths();

	// parametrize and fit to surface
	void fitting();

	// -------------------------------------
	// settings
	// -------------------------------------

	void useBaseMesh(bool use) { m_useBaseMesh = use; }
	bool useBaseMesh() const { return m_useBaseMesh; }

	void useColors(bool use) { m_useColors = use; }
	bool useColors() const { return m_useColors; }

private:

	void prepare();
	void cleanup();

	ID& id(FH fh)
	{
		return m_mesh.property(m_region, fh);
	}
	ID id(const FH fh) const
	{
		return m_mesh.property(m_region, fh);
	}

	FH& pred(FH fh)
	{
		return m_mesh.property(m_pred, fh);
	}
	FH pred(const FH fh) const
	{
		return m_mesh.property(m_pred, fh);
	}
	double& dist(FH fh)
	{
		return m_mesh.property(m_distance, fh);
	}

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

	std::pair<ID, ID>& faceBorder(FH fh)
	{
		return m_mesh.property(m_fborder, fh);
	}

	bool isBorder(const FH fh)
	{
		auto b = faceBorder(fh);
		return b.first != -1 && b.second != -1;
	}

	void setFaceBorder(const FH fh, ID id1 = -1, ID id2 = -1)
	{
		auto b = faceBorder(fh);
		b.first = id1;
		b.second = id2;
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

	bool isRegionBorderEdge(EH e) const
	{
		HH h = m_mesh.halfedge_handle(e, 0);
		return id(m_mesh.face_handle(h)) != id(m_mesh.opposite_face_handle(h));
	}

	// everythign related to the voronoi partition
	using QElem = std::pair<double, FH>;
	using Dijkstra = std::set<QElem>;

	void resetPath(Dijkstra &q, const FH face);

	void repartition(const ID id1);

	EH commonEdge(const FH f0, const FH f1)
	{
		for (auto ff = m_mesh.cfh_begin(f0); ff != m_mesh.cfh_end(f0); ++ff) {
			if (m_mesh.opposite_face_handle(*ff) == f1) {
				return m_mesh.edge_handle(*ff);
			}
		}
		return EH();
	}

	void grow(Dijkstra &q, const FH face, const FH predFace=FH(), double distance=0.0)
	{
		id(face) = !predFace.is_valid() ? m_seeds.size() - 1 : id(predFace);
		if (m_useColors) {
			m_mesh.set_color(face, m_colors[id(face)]);
		}
		pred(face) = predFace;
		if (predFace.is_valid()) assert(face != pred(pred(face)));

		auto pair = QElem(dist(face), face);
		auto it = q.find(pair);
		if (it != q.end()) q.erase(it);

		dist(face) = distance;
		pair.first = distance;
		q.insert(pair);
	};

	bool homeomorphicDisk(const FH f, const VH v, const ID tile) const
	{
		const auto isAdjTo = [&](const VH v, const ID tile) {
			for (auto f = m_mesh.cvf_begin(v); f != m_mesh.cvf_end(v); ++f) {
				if (id(*f) == tile) return true;
			}
			return false;
		};

		int countEdge = 0;
		for (auto he = m_mesh.cfh_begin(f); he != m_mesh.cfh_end(f); ++he) {
			if (id(m_mesh.opposite_face_handle(*he)) != tile) countEdge++;
		}
		return !(isAdjTo(v, tile) && countEdge == 2);
	}

	void reduceCuts(Dijkstra &q);

	void reduceAdjRegions(Dijkstra &q);

	bool isSeed(const FH f) const
	{
		if (id(f) >= 0) {
			return id(f) < m_seeds.size() && m_seeds[id(f)] == f;
		} else {
			return std::find(m_seeds.begin(), m_seeds.end(), f) != m_seeds.end();
		}
	}

	void addSeed(Dijkstra &q, const FH f)
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
		m_mesh.set_color(f, c);

		P p1 = m_mesh.calc_face_centroid(f);
		for (auto he = m_mesh.fh_begin(f); he != m_mesh.fh_end(f); ++he) {
			auto ff = m_mesh.face_handle(he);
			P p2 = m_mesh.calc_face_centroid(ff);
			double distance = (p1 - p2).norm();
			if (distance < dist(ff)) {
				grow(q, ff, f, distance);
			}
		}
	}

	static void copyMesh(BezierTMesh &src, BezierTMesh &dest);

	void preventiveEdgeSplits();

	void assignInnerVertices();

	void splitClosedPaths();

	void fixPredecessor(const FH fh, const bool rewrite=false);

	template <class Container>
	void fixAllFaces(Container &q1, Container &q2)
	{
		std::cerr << "FIX INNER FACES\n";
		for (FH face : q1) {
			if (!m_mesh.status(face).tagged()) fixPredecessor(face, true);
		}
		std::cerr << "FIX OUTER FACES\n";
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
		const ShortestPath &path
	);

	void connectPaths(const ShortestPath & p0, const ShortestPath & p1, const FH f);

	ID seedVertex(const VH vh) const
	{
		for (auto f_it = m_mesh.cvf_begin(vh); f_it != m_mesh.cvf_end(vh); ++f_it) {
			if (!pred(*f_it).is_valid()) return id(*f_it);
		}
		return -1;
	}

	FH findDelaunayFace(ID r0, ID r1, ID r2)
	{
		for (FH f : m_ctrl.faces()) {
			if (ttv(f).isRegion(r0, r1, r2)) {
				return f;
			}
		}
		return FH();
	}

	// -------------------------------------------------------------- //
	// member variables
	// -------------------------------------------------------------- //

	BezierTMesh &m_mesh, &m_ctrl;

	bool m_useColors, m_copy, m_debugCancel, m_useBaseMesh;
	size_t m_nvertices, m_nedges, m_vertexIdx;
	ACG::HaltonColors m_colGen;

	std::vector<Color> m_colors;
	std::vector<FH> m_seeds;
	std::vector<VH> m_ctrlVerts;

	// property handles
	OpenMesh::FPropHandleT<ID>			  m_region;
	OpenMesh::FPropHandleT<FH>			  m_pred;
	OpenMesh::FPropHandleT<double>		  m_distance;
	// must be something other than bool because vector<bool> is handled uniquely in C++
	OpenMesh::EPropHandleT<ID>			  m_crossed;
	//OpenMesh::FPropHandleT<FaceSplit>	  m_split;

	OpenMesh::FPropHandleT<TriToVertex>   m_ttv;
	OpenMesh::VPropHandleT<VertexToTri>	  m_vtt;

	OpenMesh::FPropHandleT<std::pair<ID,ID>>   m_fborder;
};

}
