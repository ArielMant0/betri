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



#include "BTOFFImporter.hh"

//-----------------------------------------------------------------------------

/// base class needs virtual destructor
BTOFFImporter::~BTOFFImporter(){

}

//-----------------------------------------------------------------------------

/// constructor
BTOFFImporter::BTOFFImporter() :
  btMesh_(0),
  object_(0),
  objectOptions_(0),
  maxFaceValence_(0) {}

//-----------------------------------------------------------------------------

/// add a mesh
void BTOFFImporter::addObject(BaseObject* _object)
{
	BTMeshObject*  btMeshObj = dynamic_cast<BTMeshObject*> (_object);

    if (btMeshObj) {
		btMesh_ = btMeshObj->mesh();
		object_ = _object;
	} else {
        std::cerr << "Error: Cannot add object. Type is unknown!" << std::endl;
    }
}

//-----------------------------------------------------------------------------

/// get vertex with given index
Vec3f BTOFFImporter::vertex(uint _index)
{
    return vertices_.size() > _index ? vertices_[ _index ] : Vec3f();
}

//-----------------------------------------------------------------------------

/// add texture coordinates
int BTOFFImporter::addTexCoord(const Vec2f& _coord)
{
    texCoords_.push_back(_coord);

    return texCoords_.size()-1;
}

//-----------------------------------------------------------------------------

/// add a normal
int BTOFFImporter::addNormal(const Vec3f& _normal)
{
    normals_.push_back(_normal);

    return normals_.size()-1;
}

//-----------------------------------------------------------------------------

/// add a color
int BTOFFImporter::addColor(const Vec4f& _color)
{
    colors_.push_back(_color);

    return colors_.size()-1;
}

//-----------------------------------------------------------------------------

BezierTMesh* BTOFFImporter::btMesh()
{
	return btMesh_ == 0 ? 0 : btMesh_;
}

//-----------------------------------------------------------------------------

/// set vertex texture coordinate
void BTOFFImporter::setVertexTexCoord(VertexHandle _vh, int _texCoordID)
{
    //handle triangle meshes
    if (!btMesh()) return;

    if (_texCoordID < (int)texCoords_.size()) {

        //perhaps request texCoords for the mesh
        if (!btMesh()->has_vertex_texcoords2D())
            btMesh()->request_vertex_texcoords2D();

        if (vertexMapBT_.find(_vh) != vertexMapBT_.end())
            btMesh()->set_texcoord2D(vertexMapBT_[_vh], texCoords_[_texCoordID]);
        objectOptions_ |= VERTEXTEXCOORDS;
    }
}

//-----------------------------------------------------------------------------

/// set vertex normal
void BTOFFImporter::setNormal(VertexHandle _vh, int _normalID){

    //handle triangle meshes
    if (!btMesh()) return;

    if (_normalID < (int)normals_.size()) {

        if (vertexMapBT_.find(_vh) != vertexMapBT_.end()) {
            btMesh()->set_normal(vertexMapBT_[_vh], (BezierTMesh::Point) normals_[_normalID]);
            objectOptions_ |= VERTEXNORMAL;
        }

    } else {
        std::cerr << "Error: normal ID too large" << std::endl;
    }
}

//-----------------------------------------------------------------------------

/// add a vertex with coordinate \c _point
VertexHandle BTOFFImporter::addVertex(const Vec3f& _point)
{
    vertices_.push_back( _point );

    int id = vertices_.size()-1;

    if (!btMesh()) return false;

    vertexMapBT_[id] = btMesh()->add_vertex((BezierTMesh::Point) vertices_[id]);

    return id;
}

//-----------------------------------------------------------------------------

/// add a face with indices _indices refering to vertices
int BTOFFImporter::addFace(const VHandles& _indices)
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

    BezierTMesh::FaceHandle fh = btMesh()->add_face(vertices, true);

    if (fh.is_valid()) {
        faceMapBT_.push_back(fh);
        faceIndex = faceMapBT_.size() - 1;
    } else {
        // Store non-manifold face
        invalidFaces_.push_back(vertices);
    }

    return faceIndex;
}

//-----------------------------------------------------------------------------

//#include <fstream>
void BTOFFImporter::finish()
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

bool BTOFFImporter::isBinary()
{
    return objectOptions_ & BINARY;
}

//-----------------------------------------------------------------------------

bool BTOFFImporter::hasVertexNormals()
{
    return objectOptions_ & VERTEXNORMAL;
}

//-----------------------------------------------------------------------------

bool BTOFFImporter::hasTextureCoords()
{
    return objectOptions_ & VERTEXTEXCOORDS;
}

//-----------------------------------------------------------------------------

bool BTOFFImporter::hasVertexColors()
{
    return objectOptions_ & VERTEXCOLOR;
}

//-----------------------------------------------------------------------------

bool BTOFFImporter::hasFaceColors()
{
    return objectOptions_ & FACECOLOR;
}

//-----------------------------------------------------------------------------

bool BTOFFImporter::hasOption(ObjectOptionsE _option)
{
    return objectOptions_ & _option;
}

//-----------------------------------------------------------------------------

uint BTOFFImporter::n_vertices()
{
    return vertices_.size();
}

//-----------------------------------------------------------------------------

uint BTOFFImporter::n_normals()
{
    return normals_.size();
}

//-----------------------------------------------------------------------------

uint BTOFFImporter::n_texCoords()
{
    return texCoords_.size();
}

//-----------------------------------------------------------------------------

void BTOFFImporter::reserve(unsigned int _nv, unsigned int _ne, unsigned int _nf)
{
    vertices_.reserve(_nv);
    normals_.reserve(_nv);
    texCoords_.reserve(_nv);
    colors_.reserve(_nv);

    btMesh_->reserve(_nv, _ne, _nf);
}

//-----------------------------------------------------------------------------

BaseObject* BTOFFImporter::getObject()
{
    return object_;
}

//-----------------------------------------------------------------------------

QString BTOFFImporter::path()
{
    return path_;
}

//-----------------------------------------------------------------------------

void BTOFFImporter::setPath(QString _path)
{
    path_ = _path;
}

//-----------------------------------------------------------------------------

void BTOFFImporter::setObjectOptions(ObjectOptions _options)
{
    objectOptions_ = _options;
}

//-----------------------------------------------------------------------------

void BTOFFImporter::addOption(ObjectOptionsE _option)
{
    objectOptions_ |= _option;
}

//-----------------------------------------------------------------------------

void BTOFFImporter::removeOption(ObjectOptionsE _option)
{
    if(objectOptions_ & _option) objectOptions_ -= _option;
}

//-----------------------------------------------------------------------------

BTOFFImporter::ObjectOptions& BTOFFImporter::objectOptions(){
    return objectOptions_;
}

//-----------------------------------------------------------------------------

void BTOFFImporter::setObjectName(QString _name)
{
    if (object_ != 0) object_->setName(_name);
}

//------------------------------------------------------------------------------

/// set RGBA color of vertex
void BTOFFImporter::setVertexColor(VertexHandle _vh, int _colorIndex)
{
    if (!btMesh()) return;

    if (_colorIndex < (int)colors_.size()) {

        if (vertexMapBT_.find(_vh) != vertexMapBT_.end()) {
            btMesh()->set_color(vertexMapBT_[_vh], colors_[_colorIndex]);
            objectOptions_ |= VERTEXCOLOR;
        }
    } else {
        std::cerr << "Error: Color ID too large" << std::endl;
    }
}

//------------------------------------------------------------------------------

/// set RGBA color of face
void BTOFFImporter::setFaceColor(FaceHandle _fh, int _colorIndex)
{
    if (!btMesh()) return;

    if (_colorIndex < (int)colors_.size()) {

        if (_fh < (int)faceMapBT_.size()) {
            btMesh()->set_color(faceMapBT_[_fh], colors_[_colorIndex]);
            objectOptions_ |= FACECOLOR;
        }

    }
    else {
        std::cerr << "Error: Color ID too large" << std::endl;
    }
}
