#pragma once

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

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

	using Container = std::unordered_set<HH>;

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

	//bool operator==(const ShortestPath& rhs) const
	//{
	//	return first() == rhs.first() && second() == rhs.second();
	//}

	bool add(HH he) { return m_border.insert(he).second; }

	Container& edges() { return m_border; }

	ID first() const { return m_small; }
	ID second() const { return m_big; }

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
