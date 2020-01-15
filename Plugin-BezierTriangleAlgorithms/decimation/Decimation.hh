#pragma once

#include "../common/Common.hh"
#include "DecimationFitting.hh"
#include "DecimationParametrization.hh"

namespace betri
{

/**
 * @brief Decimator that produces a decimated Bezier triangle mesh.
 *
 */
class Decimation
{
public:

	/**
	 * @brief Construct a new Decimation object
	 *
	 * @param mesh The mesh to decimate
	 */
    Decimation(BezierTMesh &mesh) :
		m_mesh(mesh),
		m_complexity(m_mesh.n_vertices()),
		m_nverts(m_mesh.n_vertices()),
		m_fittingSamples(40u),
		m_fittingSolver(Fitting::Solver::adaptive),
		m_q(nullptr),
		m_param(mesh),
		m_fit(mesh),
		m_cancel(false),
		m_interpolate(false),
		m_useColors(true)
    {
		prepare();
		// create priority q
		VertexCmp cmp(mesh);
		m_q = new std::set<VertexHandle, VertexCmp>(cmp);
    }

	/**
	 * @brief Destroy the Decimation object
	 */
    ~Decimation()
    {
        cleanup();
		if (m_q) {
			delete m_q;
			m_q = nullptr;
		}
    }

	/**
	 * @brief Initializes the object for the desired target complexity.
	 *
	 * @param complexity target vertex compelxity
	 */
	void initialize(size_t complexity);

	/**
	 * @brief Performs the decimation.
	 *
	 * @param stepwise whether to only perform 1 step
	 * @return true when the decimation is finished
	 * @return false else
	 */
    bool decimate(bool stepwise=false);

	/**
	 * @brief Sets whether interpolation should be used.
	 *
	 * If this option is enabled, instead of fixing the edge
	 * constrol points for the outer 1-ring edges, the control
	 * points for two neighboring faces are interpolated at their
	 * common edge. Usually not a better option and more work.
	 *
	 * @param use whether interpolation should be enabled
	 */
	void interpolate(bool use) { m_interpolate = use; }
	/**
	 * @brief Returns whether interpolation is enabled.
	 *
	 * @return true if interpolation is enabled
	 * @return false else
	 */
	bool interpolate() const { return m_interpolate; }

	/**
	 * @brief Sets whether to display the fitting error using vertex colors.
	 *
	 * @param use whether to display the fitting erro
	 */
	void useColors(bool use) { m_useColors = use; }
	/**
	 * @brief Returns whether color usage is enabled.
	 *
	 * @return true if color usage is enabled
	 * @return false else
	 */
	bool useColors() const { return m_useColors; }

	/**
	 * @brief Sets the desired target compexity.
	 *
	 * The value is clamped between (1,#vertices).
	 *
	 * @param target desired vertex complexity
	 */
	void complexity(size_t target)
	{
		m_complexity = std::min((size_t)1, std::max(m_mesh.n_vertices(), target));
	}
	/**
	 * @brief Returns the target vertex complexity.
	 *
	 * @return size_t target vertex complexity
	 */
	size_t complexity() const { return m_complexity; }

	/**
	 * @brief Sets the desired nummber of sample points to use
	 * during fitting.
	 *
	 * @param number nummber of samples
	 */
	void fittingSamples(size_t number) { m_fittingSamples = number; }
	/**
	 * @brief Returns the nummber of fittin samples.
	 *
	 * @return size_t
	 */
	size_t fittingSamples() const { return m_fittingSamples; }

	/**
	 * @brief Sets the desired solver for the fitting step.
	 *
	 * Possible values are:
	 * - normal equations (slow but inaccurate)
	 * - qr decomposition (more accurate but slower)
	 * - adaptive (choose one of the previous two, depending on sample count)
	 *
	 * @param solver
	 */
	void fittingSolver(Fitting::Solver solver) { m_fittingSolver = solver; }
	/**
	 * @brief Returns the used fitting solver.
	 *
	 * @return Fitting::Solver used fitting solver
	 */
	Fitting::Solver fittingSolver() const { return m_fittingSolver; }

private:

	/**
	 * @brief Comparison functor for priority queue.
	 *
	 */
	struct VertexCmp
	{
		//! mesh
		BezierTMesh *m_mesh;

		/**
		 * @brief Constructs a new Vertex Cmp object
		 *
		 * @param mesh
		 */
		VertexCmp(BezierTMesh &mesh) : m_mesh(&mesh) {}

		/**
		 * @brief Returns which vertex is smaller.
		 *
		 * This is determined using the vertex priority (aka min error).
		 * Should both have the same error, the vertex handle's index is used
		 * for comparison, because a unique ordering is required.
		 *
		 * @param v0
		 * @param v1
		 * @return true if vertex v0 is smaller
		 * @return false else
		 */
		bool operator()(const VertexHandle v0, const VertexHandle v1) const
		{
			assert(m_mesh != nullptr);
			OpenMesh::VPropHandleT<double> vprio;
			m_mesh->get_property_handle(vprio, "vertexprio");

			Scalar p0 = m_mesh->property(vprio, v0),
				p1 = m_mesh->property(vprio, v1);
			// std::set needs UNIQUE keys -> handle equal priorities
			return p0 == p1 ? v0.idx() < v1.idx() : p0 < p1;
		}
	};

	//-----------------------------------------------//
	// member functions
	//-----------------------------------------------//

	/**
	 * @brief Performs one decimation step, a halfedge collapse
	 * for the halfedge with the smallest estimated error.
	 */
	void step();

	/**
	 * @brief Performs preparations.
	 */
    void prepare();
	/**
	 * @brief Performs clean up.
	 */
    void cleanup();

	/**
	 * @brief Calculates parametrization and fitting for the
	 * halfedge collapse of halfedge hh.
	 *
	 * This method is used to both estimate the error and to
	 * perform fitting after a halfedge is collapsed.
	 *
	 * @param hh halfedge to collapse
	 * @param apply whether to apply the fitting (or only estimate the error)
	 * @return Scalar the estimated error
	 */
	Scalar fit(const HalfedgeHandle hh, const bool apply);

	/**
	 * @brief Calculates fitting error estimate for all halfedges and vertices.
	 */
	void calculateErrors();
		/**
	 * @brief Calculates fitting error estimate for one halfedge.
	 */
	void calculateError(const HalfedgeHandle he);
	/**
	 * @brief Updates the error for a specific halfedge by adding the
	 * specified error and estimating its own error again.
	 *
	 * @param he halfedge
	 * @param add erro to add on top of the estimation
	 */
	void updateError(const HalfedgeHandle he, const Scalar add);

	/**
	 * @brief Returns a reference to the target halfedge for a
	 * given vertex.
	 *
	 * The target halfedge is the one that determines the vertex'
	 * error value.
	 *
	 * @param vh vertex handle
	 * @return HalfedgeHandle& halfedge handle
	 */
	HalfedgeHandle& target(const VertexHandle vh) { return m_mesh.property(m_target, vh); }

	/**
	 * @brief Adds a vertex to the queue (or updates in position in it).
	 *
	 * @param vh
	 */
	void enqueueVertex(const VertexHandle vh);

	/**
	 * @brief Checks whether a halfedge collapse is legal and returns
	 * a multiplier with which to weigh the error estimate.
	 *
	 * @param hh halfedge
	 * @return double multiplier for error in [0,1]
	 */
	double isCollapseLegal(const HalfedgeHandle hh);

	/**
	 * @brief Returns the error estimate of a halfedge.
	 *
	 * @param hh
	 * @return Scalar&
	 */
	Scalar& priority(const HalfedgeHandle hh) { return m_mesh.property(m_hprio, hh); }
	/**
	 * @brief Returns the error estimate of a vertex.
	 *
	 * @param hh
	 * @return Scalar&
	 */
	Scalar& priority(const VertexHandle vh) { return m_mesh.property(m_vprio, vh); }

	/**
	 * @brief Calculates the vertex priority (min error).
	 *
	 * @param vh
	 * @return Scalar
	 */
	Scalar calcVertexPriority(const VertexHandle vh)
	{
		Scalar min = std::numeric_limits<Scalar>::max();
		HalfedgeHandle hh;

		for (auto h_it = m_mesh.cvoh_begin(vh); h_it != m_mesh.cvoh_end(vh); ++h_it) {
			if (priority(*h_it) < min) {
				min = priority(*h_it);
				hh = *h_it;
			}
		}

		return min;
	}

	/**
	 * @brief Calculates and saves the min, max and mean eror values.
	 */
	void calcErrorStatistics();
	/**
	 * @brief Set vertex colors from their error values.	 *
	 */
	void setColorsFromError();

	/**
	 * @brief Sets a flag that the algorithm should be cancelled
	 * at the next convenient location and stores an error message.
	 *
	 * @param msg the error message
	 */
	void debugCancel(const char *msg)
	{
		m_cancel = true;
		m_errors += msg;
		m_errors += '\n';
	}
	/**
	 * @brief Sets a flag that the algorithm should be cancelled
	 * at the next convenient location and stores an error message.
	 *
	 * @param msg the error message
	 */
	void debugCancel(const std::string &msg)
	{
		m_cancel = true;
		m_errors += msg;
		m_errors += '\n';
	}
	/**
	 * @brief Aks whether the algorithm should be cancelled.
	 * If that is the case, error messages are printed to the console.
	 *
	 * @return true cancel the algorithm
	 * @return false else
	 */
	bool debugCancel()
	{
		if (m_cancel && !m_errors.empty()) {
			std::cerr << "------------------------------\n";
			std::cerr << "Decimation Error:\n" << m_errors << std::endl;
			std::cerr << "------------------------------\n";
			m_errors.clear();
		}
		return m_cancel;
	}

	//-----------------------------------------------//
	// member variables
	//-----------------------------------------------//

	//! mesh to decimate
	BezierTMesh &m_mesh;

	//! object to calculate the fitting
	DecimationFitting m_fit;
	//! object to calculate the parametrization
	DecimationParametrization m_param;

	//! accumulated error messages
	std::string m_errors;
	//! whether to cancel execution
	bool m_cancel;
	//! whether to interpolate instead of fixing control points
	bool m_interpolate;
	//! whether to display the error values using vertex colors
	bool m_useColors;

	//! desired complexity and current vertex count
	size_t m_complexity;
	//! current number of vertices
	size_t m_nverts;
	//! number of fitting samples to use
	size_t m_fittingSamples;
	//! fitting solver to use
	Fitting::Solver m_fittingSolver;

	//! minimum error for any halfedge (not counting invalid collapses with a value if -1)
	Scalar m_minError;
	//! avergae error for any halfedge
	Scalar m_avgError;
	//! maximum error for any halfedge
	Scalar m_maxError;

	//! priority queue
	std::set<VertexHandle, VertexCmp> *m_q;

	//! property handle for halfedge error
	OpenMesh::HPropHandleT<Scalar> m_hprio;
	//! property handle for vertex error
	OpenMesh::VPropHandleT<Scalar> m_vprio;
	//! property handle for the target halfedge for a vertex
	OpenMesh::VPropHandleT<HalfedgeHandle> m_target;
};

}
