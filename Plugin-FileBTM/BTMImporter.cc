/*===========================================================================*\
*                                                                            *
*                              OpenFlipper                                   *
 *           Copyright (c) 2001-2015, RWTH-Aachen University                 *
 *           Department of Computer Graphics and Multimedia                  *
 *                          All rights reserved.                             *
 *                            www.openflipper.org                            *
 *                                                                           *
 *---------------------------------------------------------------------------*
 * This file is part of OpenFlipper.                                         *
 *---------------------------------------------------------------------------*
 *                                                                           *
 * Redistribution and use in source and binary forms, with or without        *
 * modification, are permitted provided that the following conditions        *
 * are met:                                                                  *
 *                                                                           *
 * 1. Redistributions of source code must retain the above copyright notice, *
 *    this list of conditions and the following disclaimer.                  *
 *                                                                           *
 * 2. Redistributions in binary form must reproduce the above copyright      *
 *    notice, this list of conditions and the following disclaimer in the    *
 *    documentation and/or other materials provided with the distribution.   *
 *                                                                           *
 * 3. Neither the name of the copyright holder nor the names of its          *
 *    contributors may be used to endorse or promote products derived from   *
 *    this software without specific prior written permission.               *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED *
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A           *
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER *
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  *
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,       *
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR        *
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    *
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING      *
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS        *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.              *
*                                                                            *
\*===========================================================================*/

#include "BTMImporter.hh"

//-----------------------------------------------------------------------------

/// base class needs virtual destructor
BTMImporter::~BTMImporter() {}

//-----------------------------------------------------------------------------

/// constructor
BTMImporter::BTMImporter() :
  btMesh_(0),
  object_(0),
  objectOptions_(0),
  maxFaceValence_(0) {}

//-----------------------------------------------------------------------------

/// add a mesh
void BTMImporter::addObject(BaseObject* _object, unsigned int degree)
{
	BTMeshObject*  btMeshObj = dynamic_cast<BTMeshObject*> (_object);

    if (btMeshObj) {
		btMesh_ = btMeshObj->mesh();
		btMesh_->degree(degree);
		object_ = _object;
	} else {
        std::cerr << "Error: Cannot add object. Type is unknown!" << std::endl;
    }
}

//-----------------------------------------------------------------------------

/// get vertex with given index
Vec3f BTMImporter::vertex(uint _index)
{
    return vertices_.size() > _index ? vertices_[ _index ] : Vec3f();
}

//-----------------------------------------------------------------------------

BezierTMesh* BTMImporter::btMesh()
{
	return btMesh_ == 0 ? 0 : btMesh_;
}

//-----------------------------------------------------------------------------

/// add a vertex with coordinate \c _point
VertexHandle BTMImporter::addVertex(const Vec3f& _point)
{
    vertices_.push_back(_point);

    int id = vertices_.size()-1;

    if (!btMesh()) return false;

    vertexMapBT_[id] = btMesh()->add_vertex((BezierTMesh::Point) vertices_[id]);

    return id;
}

//-----------------------------------------------------------------------------

/// add a face with indices _indices refering to vertices
int BTMImporter::addFace(const VHandles& _indices)
{
    int faceIndex = -1;

    //handle triangle meshes
    if (!btMesh()) return -1;

    std::vector<BezierTMesh::VertexHandle> vertices;

    for (uint i = 0; i < _indices.size(); i++) {

        if (vertexMapBT_.find(_indices[i]) != vertexMapBT_.end()) {
            vertices.push_back(vertexMapBT_[_indices[i]]);
        } else {
            std::cerr << "Error: Cannot add face. Undefined index (" << _indices[i] << ")" << std::endl;
            return -1;
        }
    }

    BezierTMesh::FaceHandle fh = btMesh()->add_face(vertices);

    if (fh.is_valid()) {
        faceMapBT_.push_back(fh);
        faceIndex = faceMapBT_.size() - 1;
    } else {
        // Store non-manifold face
        invalidFaces_.push_back(vertices);
    }

    return faceIndex;
}

void BTMImporter::addControlPoint(BezierTMesh::FaceHandle fh, float x, float y, float z)
{
    btMesh()->data(fh).addPoint(BezierTMesh::Point(x, y, z));
}

//-----------------------------------------------------------------------------

//#include <fstream>
void BTMImporter::finish()
{
	if (invalidFaces_.empty()) {
		return;
	}

    for (std::vector<OMVHandles>::iterator it = invalidFaces_.begin();
        it != invalidFaces_.end(); ++it) {

        OMVHandles& vhandles = *it;

        // double vertices
        for (unsigned int j = 0; j < vhandles.size(); ++j)
        {
            BezierTMesh::Point p = btMesh()->point(vhandles[j]);
            vhandles[j] = btMesh()->add_vertex(p);
            // DO STORE p, reference may not work since vertex array
            // may be relocated after adding a new vertex !

            // Mark vertices of failed face as non-manifold
            if (btMesh()->has_vertex_status()) {
                btMesh()->status(vhandles[j]).set_fixed_nonmanifold(true);
            }
        }

        // TODO: is this correct?
        // add face
        OpenMesh::FaceHandle fh = btMesh()->add_face(vhandles, true);

        // Mark failed face as non-manifold
        if (btMesh()->has_face_status())
            btMesh()->status(fh).set_fixed_nonmanifold(true);

        // Mark edges of failed face as non-two-manifold
        if (btMesh()->has_edge_status()) {
            BezierTMesh::FaceEdgeIter fe_it = btMesh()->fe_iter(fh);
            for (; fe_it.is_valid(); ++fe_it) {
                btMesh()->status(*fe_it).set_fixed_nonmanifold(true);
            }
        }

        faceMapBT_.push_back(fh);
    }

    // Clear all invalid faces
    invalidFaces_.clear();
}

//-----------------------------------------------------------------------------

bool BTMImporter::isBinary()
{
    return objectOptions_ & BINARY;
}

//-----------------------------------------------------------------------------

bool BTMImporter::hasOption(ObjectOptionsE _option)
{
    return objectOptions_ & _option;
}

//-----------------------------------------------------------------------------

uint BTMImporter::n_vertices()
{
    return vertices_.size();
}

//-----------------------------------------------------------------------------

void BTMImporter::reserve(unsigned int _nv, unsigned int _ne, unsigned int _nf)
{
    vertices_.reserve(_nv);

    btMesh_->reserve(_nv, _ne, _nf);
}

//-----------------------------------------------------------------------------

BaseObject* BTMImporter::getObject()
{
    return object_;
}

//-----------------------------------------------------------------------------

QString BTMImporter::path()
{
    return path_;
}

//-----------------------------------------------------------------------------

void BTMImporter::setPath(QString _path)
{
    path_ = _path;
}

//-----------------------------------------------------------------------------

void BTMImporter::setObjectOptions(ObjectOptions _options)
{
    objectOptions_ = _options;
}

//-----------------------------------------------------------------------------

void BTMImporter::addOption(ObjectOptionsE _option)
{
    objectOptions_ |= _option;
}

//-----------------------------------------------------------------------------

void BTMImporter::removeOption(ObjectOptionsE _option)
{
    if(objectOptions_ & _option) objectOptions_ -= _option;
}

//-----------------------------------------------------------------------------

BTMImporter::ObjectOptions& BTMImporter::objectOptions(){
    return objectOptions_;
}

//-----------------------------------------------------------------------------

void BTMImporter::setObjectName(QString _name)
{
    if (object_ != 0) object_->setName(_name);
}

//------------------------------------------------------------------------------
