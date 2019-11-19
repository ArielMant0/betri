#pragma once

#include "../common/Common.hh"

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

	using Container = std::vector<VH>;

	ShortestPath() : m_small(-1), m_big(-1), m_start(-1), m_border() {}

	ShortestPath(const ShortestPath &other) = default;

	ShortestPath(ID id1, ID id2) : m_start(id1), m_border()
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
		m_start = *l.begin();
		assert(m_small < m_big);
	}

	void push(VH vh) const { return m_border.push_back(vh); }

	void pop() const { m_border.pop_back(); }

	Container& list() const { return m_border; }
	Container& list(ID start) const
	{
		if (m_start != start) {
			std::reverse(m_border.begin(), m_border.end());
			m_start = start;
		}

		return m_border;
	}
	Container& list(const ShortestPath &path) const
	{
		if (path.start() == start()) {
			reverse();
		} else if (path.end() == end()) {
			path.reverse();
		} else if (path.end() == start()) {
			reverse();
			path.reverse();
		}

		return m_border;
	}

	void reverse() const
	{
		std::reverse(m_border.begin(), m_border.end());
		m_start = m_start == m_small ? m_big : m_small;
	}

	bool contains(VH e) const
	{
		return std::any_of(m_border.begin(), m_border.end(), [e](const VH eh) {
			return eh == e;
		});
	}

	bool partOf(const ID id) const { return m_small == id || m_big == id; }

	bool replace(
		const BezierTMesh &mesh,
		const VH adj,
		const VH now
	);

	ID first() const { return m_small; }
	ID second() const { return m_big; }

	ID start() const { return m_start;  }
	void start(ID newStart)
	{
		assert(newStart == m_small || newStart == m_big);
		m_start = newStart;
	}
	ID end() const { return m_small == m_start ? m_big : m_small;  }

	VH front() const { return m_border[0]; }
	VH back() const { return m_border.back(); }

	size_t size() const { return m_border.size(); }

	static const ShortestPath& path(ID id1, ID id2);
	static void path(ShortestPath &s);

	static std::unordered_set<betri::ShortestPath>* pathList();

	static bool has(const ID id1, const ID id2);

	static void connect(BezierTMesh &mesh, const ID id1, const ID id2, const ID id3);

	static void replace(
		const BezierTMesh &mesh,
		const VH adj,
		const VH now,
		const ID hint1,
		const ID hint2
	);

	static void clear();

private:

	ID m_small, m_big;

	mutable ID m_start; // with which id the list starts
	mutable Container m_border; // the halfedges that make up the border
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
