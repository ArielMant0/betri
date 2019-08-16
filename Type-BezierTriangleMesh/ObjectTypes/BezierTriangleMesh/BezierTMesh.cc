#include "BezierTMesh.hh"

using FaceHandle = BezierTMesh::FaceHandle;

// TODO:: make it work for variable degree
void BezierTMesh::addCPsToFace(FaceHandle &f, unsigned int degree)
{
	auto v_it = cfv_begin(f);
	auto p1 = point(*v_it); v_it++;
	auto p2 = point(*v_it); v_it++;
	auto p3 = point(*v_it);

	auto bt = data(f);
	bt.degree(degree);
	bt.addPoint(p1);
	bt.addPoint(p1 * 0.5f + p2 * 0.5f);
	bt.addPoint(p2);
	bt.addPoint(p2 * 0.5f + p3 * 0.5f);
	bt.addPoint(p3);
	bt.addPoint(p3 * 0.5f + p1 * 0.5f);
}

void BezierTMesh::addCPsToFace(const FaceHandle &f, unsigned int degree)
{
	auto v_it = cfv_begin(f);
	auto p1 = point(*v_it); v_it++;
	auto p2 = point(*v_it); v_it++;
	auto p3 = point(*v_it);

	auto bt = data(f);
	bt.degree(degree);
	bt.addPoint(p1);
	bt.addPoint(p1 * 0.5f + p2 * 0.5f);
	bt.addPoint(p2);
	bt.addPoint(p2 * 0.5f + p3 * 0.5f);
	bt.addPoint(p3);
	bt.addPoint(p3 * 0.5f + p1 * 0.5f);
}