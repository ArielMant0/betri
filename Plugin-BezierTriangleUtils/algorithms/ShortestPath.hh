#pragma once

#include "Common.hh"

#include <vector>
#include <unordered_set>


namespace betri
{

class ShortestPath
{
public:

	using ID = int;
	using VH = BezierTMesh::VertexHandle;
	using EH = BezierTMesh::EdgeHandle;
	using HH = BezierTMesh::HalfedgeHandle;
	using FH = BezierTMesh::FaceHandle;

	using Container = std::vector<EH>;

	ShortestPath() : m_small(-1), m_big(-1), m_border() {}

	ShortestPath(const ShortestPath &other) = default;

	ShortestPath(ID id1, ID id2) : m_border()
	{
		if (id1 < id2) {
			m_small = id1;
			m_big = id2;
		} else {
			m_small = id2;
			m_big = id1;
		}
	}

	ShortestPath(std::initializer_list<ID> l) : m_border()
	{
		assert(l.size() >= 2);
		m_small = *l.begin();
		m_big = *std::next(l.begin());
		if (m_small > m_big) {
			ID tmp = m_small;
			m_small = m_big;
			m_big = tmp;
		}
		assert(m_small < m_big);
	}

	void push(EH he) { return m_border.push_back(he); }

	void pop() { m_border.pop_back(); }

	Container& edges() { return m_border; }

	bool contains(BezierTMesh &mesh, HalfedgeHandle he) const
	{
		return std::any_of(m_border.begin(), m_border.end(), [he,&mesh](const EdgeHandle eh) {
			return mesh.edge_handle(he) == eh;
		});
	}

	bool contains(EdgeHandle e) const
	{
		return std::any_of(m_border.begin(), m_border.end(), [e](const EdgeHandle eh) {
			return eh == e;
		});
	}

	bool partOf(const ID id) const { return m_small == id || m_big == id; }

	bool replace(const BezierTMesh &mesh, const VH adj, const VH now, std::function<void(EH)> func);

	ID first() const { return m_small; }
	ID second() const { return m_big; }

	static ShortestPath path(ID id1, ID id2);
	static void path(ShortestPath &s);

	static bool has(const ID id1, const ID id2);

	static void replace(
		const BezierTMesh &mesh,
		const VH adj,
		const VH now,
		const ID hint1,
		const ID hint2,
		std::function<void(EH)> func
	);

	static void clear();

private:

	ID m_small, m_big;
	Container m_border; // the halfedges that make up the border
};

inline bool operator==(const ShortestPath& lhs, const ShortestPath& rhs)
{
	return lhs.first() == rhs.first() && lhs.second() == rhs.second();
}

}

namespace std
{
template<> struct hash<betri::ShortestPath>
{
	typedef betri::ShortestPath argument_type;
	typedef std::size_t result_type;
	result_type operator()(argument_type const& s) const noexcept
	{
		return std::hash<betri::ShortestPath::ID>{}(s.first());
	}
};
}