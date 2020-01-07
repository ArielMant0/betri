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

#pragma once


//=== INCLUDES ================================================================


// STL
#include <vector>

// OpenMesh
#include <OpenFlipper/common/GlobalDefines.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>
#include <OpenFlipper/common/BaseObject.hh>

//=== IMPLEMENTATION ==========================================================

using VertexHandle = int;
using FaceHandle = int;
using VHandles = std::vector<VertexHandle>;
using OMVHandles = std::vector<OpenMesh::VertexHandle>;
using Vec4f = OpenMesh::Vec4f;
using Vec3f = OpenMesh::Vec3f;
using Vec2f = OpenMesh::Vec2f;
using Vec4uc = OpenMesh::Vec4uc;
using Vec3uc = OpenMesh::Vec3uc;


class BTMImporter
{
public:

    enum ObjectOptionsE
    {
        NONE = 0,
        BINARY = 1,
        VERTEXNORMAL = 1 << 1,
        VERTEXTEXCOORDS = 1 << 2,
        VERTEXCOLOR = 1 << 3,
        FACECOLOR = 1 << 4,
        COLORALPHA = 1 << 5,
        FORCE_NOCOLOR = 1 << 6,
        FORCE_NONORMALS = 1 << 7,
        FORCE_NOTEXTURES = 1 << 8
    };

    using ObjectOptions = uint;

    /// constructor
    BTMImporter();

    /// base class needs virtual destructor
    ~BTMImporter();

    /// add initial object
    void addObject(BaseObject* _object, unsigned int degree);

    unsigned int maxFaceValence() const { return maxFaceValence_; }

    void maxFaceValence(unsigned int _maxValence) { maxFaceValence_ = _maxValence; }

    /// add a vertex with coordinate \c _point
    VertexHandle addVertex(const Vec3f& _point);

    /// get vertex with given index
    Vec3f vertex(uint _index);

    /// get a pointer to the active mesh
	BezierTMesh* btMesh();

    /// add a face with indices _indices refering to vertices
    int addFace(const VHandles& _indices);

	BezierTMesh::FaceHandle face(int index)
	{
		return faceMapBT_[index];
	}

    void addControlPoint(BezierTMesh::FaceHandle fh, float x, float y, float z);

    /// Query Object Options
    bool isBinary();

    /// Global Properties
    uint n_vertices();

    // Reserve memory for all entity types
    void reserve(unsigned int _nv, unsigned int _ne, unsigned int _nf);

    /// Path of the OFF file
    QString path();
    void setPath(QString _path);

    /// store an initial options object for an object
    /// containing info about the meshType
    void setObjectOptions(ObjectOptions _options);

    /// add an option
    void addOption(ObjectOptionsE _option);

    /// remove an option
    void removeOption(ObjectOptionsE _option);

    /// test if object has a certain option
    bool hasOption(ObjectOptionsE _option);

    /// get Object Options
    ObjectOptions& objectOptions();

    /// change the name of an object
    void setObjectName(QString _name);

    /// get BaseObject data of object
    BaseObject* getObject();

    /// Finish up importing process:
    /// Duplicate vertices of non-manifold faces and
    /// add these faces as isolated ones
    void finish();

  private:

public:
    // general data
    std::vector<Vec3f> vertices_;

    // file path
    QString path_;

	// bezier triangle mesh data
	std::map<int, BezierTMesh::VertexHandle> vertexMapBT_;

	std::vector<BezierTMesh::FaceHandle> faceMapBT_;

	BezierTMesh* btMesh_;

private:
    //object data
    BaseObject* object_;
    ObjectOptions objectOptions_;

    // Store invalid face vertex handles
    std::vector<OMVHandles> invalidFaces_;

    // Keep track of max face valence
    unsigned int maxFaceValence_;
    unsigned int cpPerFace_;
};

