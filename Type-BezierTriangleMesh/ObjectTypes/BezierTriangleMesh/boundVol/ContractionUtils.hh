//	Copyright (c) 2013, Andre Gaschler, Quirin Fischer
//	All rights reserved.
//
//	Redistribution and use in source and binary forms, with or without
// modification,
//	are permitted provided that the following conditions are met:
//
//	* Redistributions of source code must retain the above copyright notice,
// this
//	  list of conditions and the following disclaimer.
//
//	* Redistributions in binary form must reproduce the above copyright
// notice, this
//	  list of conditions and the following disclaimer in the documentation
// and/or
//	  other materials provided with the distribution.
//
//	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND
//	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED
//	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR
//	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES
//	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES;
//	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON
//	ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS
//	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <set>
#include <Eigen/StdVector>

#include "../BezierTMesh.hh"

namespace boundingmesh
{
using Real = double;
using Vector3 = Eigen::Matrix<Real, 3, 1>;
using Vector4 = Eigen::Matrix<Real, 4, 1>;
using Matrix33 = Eigen::Matrix<Real, 3, 3>;
using Matrix44 = Eigen::Matrix<Real, 4, 4>;
//using Index = unsigned int;

class EdgeContraction
{
public:
	EdgeContraction();
	EdgeContraction(BezierTMesh::EdgeHandle edge, BezierTMesh::Point new_point, Real cost,
		const Matrix44& qem);

	BezierTMesh::EdgeHandle edge() const;
	BezierTMesh::Point new_point() const;
	Real cost() const;
	const Matrix44& qem() const;

	friend bool operator<(const EdgeContraction& lhs, const EdgeContraction& rhs);
	friend bool operator>(const EdgeContraction& lhs, const EdgeContraction& rhs);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	BezierTMesh::EdgeHandle edge_;
	BezierTMesh::Point new_point_;
	Real cost_;
	Matrix44 qem_;
};

typedef std::multiset<EdgeContraction, std::less<EdgeContraction>,
	Eigen::aligned_allocator<EdgeContraction>>::iterator
	ContractionIterator;

class ContractionIndex
{
public:
	ContractionIndex(BezierTMesh::EdgeHandle index);
	ContractionIndex(BezierTMesh::EdgeHandle index, ContractionIterator contraction);

	ContractionIterator contraction() const;

	friend bool operator<(const ContractionIndex& lhs,
		const ContractionIndex& rhs);
	friend bool operator>(const ContractionIndex& lhs,
		const ContractionIndex& rhs);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	BezierTMesh::EdgeHandle index_;
	ContractionIterator contraction_;
	bool searching_;
};

class ContractionQueue
{
public:
	ContractionQueue();
	unsigned int size();
	const EdgeContraction& first();
	void insert(const EdgeContraction& contraction);
	void remove(BezierTMesh::EdgeHandle edge);

	friend void swap(ContractionQueue& first, ContractionQueue& second);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	std::multiset<EdgeContraction, std::less<EdgeContraction>,
		Eigen::aligned_allocator<EdgeContraction> >
		contractions_;
	std::set<ContractionIndex> indices_;

	friend class Decimator;
};
}  // namespace boundingmesh