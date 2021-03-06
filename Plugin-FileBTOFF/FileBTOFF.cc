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

#include "FileBTOFF.hh"

#include <OpenFlipper/Utils/FileIO/NumberParsing.hh>
#include <OpenFlipper/Utils/Memory/RAMInfo.hh>
#include <OpenFlipper/common/GlobalOptions.hh> // TODO: ?
#include <OpenFlipper/BasePlugin/PluginFunctions.hh>  // TODO: ?

#include <QMessageBox>
#include <QVBoxLayout>

using namespace Utils;

//-----------------------------------------------------------------------------
// help functions

/// Constructor
FileBTOFFPlugin::FileBTOFFPlugin() :
    loadOptions_(0),
    saveOptions_(0),
    saveBinary_(0),
    saveVertexColor_(0),
    saveFaceColor_(0),
    saveAlpha_(0),
    saveNormals_(0),
    saveTexCoords_(0),
    savePrecisionLabel_(0),
    savePrecision_(0),
    saveDefaultButton_(0),
    loadVertexColor_(0),
    loadFaceColor_(0),
    loadAlpha_(0),
    loadNormals_(0),
    loadTexCoords_(0),
    loadCheckManifold_(0),
    loadDefaultButton_(0),
    userReadOptions_(0),
    userWriteOptions_(0),
    readColorComp_(false),
    trimeshOptions_(BTOFFImporter::NONE) {}

//-----------------------------------------------------------------------------------------------------

void FileBTOFFPlugin::initializePlugin() {
    // Initialize standard options that can then be changed in the file dialogs
    if (OpenFlipperSettings().value("FileBTOff/Load/VertexColor",true).toBool())
        userReadOptions_ |= BTOFFImporter::VERTEXCOLOR;
    if (OpenFlipperSettings().value("FileBTOff/Load/FaceColor",true).toBool())
        userReadOptions_ |= BTOFFImporter::FACECOLOR;
    if (OpenFlipperSettings().value("FileBTOff/Load/Alpha",true).toBool())
        userReadOptions_ |= BTOFFImporter::COLORALPHA;
    if (OpenFlipperSettings().value("FileBTOff/Load/Normal",true).toBool())
        userReadOptions_ |= BTOFFImporter::VERTEXNORMAL;
    if (OpenFlipperSettings().value("FileBTOff/Load/TexCoords",true).toBool())
        userReadOptions_ |= BTOFFImporter::VERTEXTEXCOORDS;

    if (OpenFlipperSettings().value("FileBTOff/Save/Binary",true).toBool())
        userWriteOptions_ |= BTOFFImporter::BINARY;
    if (OpenFlipperSettings().value("FileBTOff/Save/VertexColor",true).toBool())
        userWriteOptions_ |= BTOFFImporter::VERTEXCOLOR;
    if (OpenFlipperSettings().value("FileBTOff/Save/FaceColor",true).toBool())
        userWriteOptions_ |= BTOFFImporter::FACECOLOR;
    if (OpenFlipperSettings().value("FileBTOff/Save/Alpha",true).toBool())
        userWriteOptions_ |= BTOFFImporter::COLORALPHA;
    if (OpenFlipperSettings().value("FileBTOff/Save/Normal",true).toBool())
        userWriteOptions_ |= BTOFFImporter::VERTEXNORMAL;
    if (OpenFlipperSettings().value("FileBTOff/Save/TexCoords",true).toBool())
        userWriteOptions_ |= BTOFFImporter::VERTEXTEXCOORDS;
}

//-----------------------------------------------------------------------------------------------------

QString FileBTOFFPlugin::getLoadFilters()
{
    return QString(tr("Object File Format files ( *.off )"));
};

//-----------------------------------------------------------------------------------------------------

QString FileBTOFFPlugin::getSaveFilters()
{
    return QString(tr("Object File Format files ( *.off )"));
};

//-----------------------------------------------------------------------------------------------------

DataType FileBTOFFPlugin::supportedType()
{
    return DATA_BEZIER_TRIANGLE_MESH;
}

//-----------------------------------------------------------------------------------------------------

void FileBTOFFPlugin::trimString(std::string& _string)
{
    // Trim Both leading and trailing spaces
    size_t start = _string.find_first_not_of(" \t\r\n");
    size_t end = _string.find_last_not_of(" \t\r\n");

    if ((std::string::npos == start ) || (std::string::npos == end)) {
        _string = "";
    } else {
        _string = _string.substr(start, end-start+1);
    }
}

//-----------------------------------------------------------------------------------------------------

bool FileBTOFFPlugin::getCleanLine( std::istream& ifs , std::string& _string, bool _skipEmptyLines)
{
    // while we are not at the end of the file
    while (true) {
        // get the current line:
        std::getline(ifs,_string);
        // Remove whitespace at beginning and end
        trimString(_string);

        // Check if string is not empty ( otherwise we continue
        if (_string.size() != 0) {
            // Check if string is a comment ( starting with # )
            if (_string[0] != '#') {
                return true;
            }
        } else if ( !_skipEmptyLines )
            return true;
    }

    if (ifs.eof()) {
        std::cerr << "End of file reached while searching for input!" << std::endl;
        return false;
    }

    return false;
}

//-----------------------------------------------------------------------------------------------------

void FileBTOFFPlugin::updateUserOptions()
{
    // If the options dialog has not been initialized, keep
    // the initial values

    if (OpenFlipper::Options::nogui() )
        return;

    // Load options
    if (loadVertexColor_) {
        if (loadVertexColor_->isChecked())
            userReadOptions_ |= BTOFFImporter::VERTEXCOLOR;
        else if (userReadOptions_ & BTOFFImporter::VERTEXCOLOR)
            userReadOptions_ -= BTOFFImporter::VERTEXCOLOR;
    }
    if (loadFaceColor_) {
        if (loadFaceColor_->isChecked())
            userReadOptions_ |= BTOFFImporter::FACECOLOR;
        else if (userReadOptions_ & BTOFFImporter::FACECOLOR)
            userReadOptions_ -= BTOFFImporter::FACECOLOR;
    }
    if (loadAlpha_) {
        if (loadAlpha_->isChecked())
            userReadOptions_ |= BTOFFImporter::COLORALPHA;
        else if (userReadOptions_ & BTOFFImporter::COLORALPHA)
            userReadOptions_ -= BTOFFImporter::COLORALPHA;
    }
    if (loadNormals_) {
        if (loadNormals_->isChecked())
            userReadOptions_ |= BTOFFImporter::VERTEXNORMAL;
        else if (userReadOptions_ & BTOFFImporter::VERTEXNORMAL)
            userReadOptions_ -= BTOFFImporter::VERTEXNORMAL;
    }
    if (loadTexCoords_) {
        if (loadTexCoords_->isChecked())
            userReadOptions_ |= BTOFFImporter::VERTEXTEXCOORDS;
        else if (userReadOptions_ & BTOFFImporter::VERTEXTEXCOORDS)
            userReadOptions_ -= BTOFFImporter::VERTEXTEXCOORDS;
    }

    // Save options
    if (saveBinary_) {
        if (saveBinary_->isChecked())
            userWriteOptions_ |= BTOFFImporter::BINARY;
        else if(userWriteOptions_ & BTOFFImporter::BINARY)
            userWriteOptions_ -= BTOFFImporter::BINARY;
    }
    if (saveVertexColor_) {
        if (saveVertexColor_->isChecked())
            userWriteOptions_ |= BTOFFImporter::VERTEXCOLOR;
        else if (userWriteOptions_ & BTOFFImporter::VERTEXCOLOR)
            userWriteOptions_ -= BTOFFImporter::VERTEXCOLOR;
    }
    if (saveFaceColor_) {
        if (saveFaceColor_->isChecked())
            userWriteOptions_ |= BTOFFImporter::FACECOLOR;
        else if (userWriteOptions_ & BTOFFImporter::FACECOLOR)
            userWriteOptions_ -= BTOFFImporter::FACECOLOR;
    }
    if (saveAlpha_) {
        if (saveAlpha_->isChecked())
            userWriteOptions_ |= BTOFFImporter::COLORALPHA;
        else if (userWriteOptions_ & BTOFFImporter::COLORALPHA)
            userWriteOptions_ -= BTOFFImporter::COLORALPHA;
    }
    if (saveNormals_) {
        if (saveNormals_->isChecked())
            userWriteOptions_ |= BTOFFImporter::VERTEXNORMAL;
        else if (userWriteOptions_ & BTOFFImporter::VERTEXNORMAL)
            userWriteOptions_ -= BTOFFImporter::VERTEXNORMAL;
    }
    if (saveTexCoords_) {
        if (saveTexCoords_->isChecked())
            userWriteOptions_ |= BTOFFImporter::VERTEXTEXCOORDS;
        else if (userWriteOptions_ & BTOFFImporter::VERTEXTEXCOORDS)
            userWriteOptions_ -= BTOFFImporter::VERTEXTEXCOORDS;
    }
}

//-----------------------------------------------------------------------------------------------------

bool FileBTOFFPlugin::readFileOptions(QString _filename, BTOFFImporter& _importer)
{
    /* Constitution of an OFF-file
       ==================================================================
       [ST] [C] [N] [4][n]OFF [BINARY] # comment
       nV nF nE # number of vertices, faces and edges (edges are skipped)
       v[0] v[1] v[2] [n[0] n[1] n[2]] [c[0] c[1] c[1]] [t[0] t[0]]
       ...
       faceValence vIdx[0] ... vIdx[faceValence-1] colorspec
       ...
       ==================================================================
    */

    const unsigned int LINE_LEN = 4096;

    std::ifstream ifs(_filename.toUtf8(), std::ios_base::binary);

    if ((!ifs.is_open()) || (!ifs.good())) {
        emit log(LOGERR, tr("Error: Could not read file options of specified OFF-file! Aborting."));
        return false;
    }

    // read 1st line
    char line[LINE_LEN], *p;
    ifs.getline(line, LINE_LEN);
    p = line;

    int remainingChars = ifs.gcount();

    // check header: [ST][C][N][4][n]OFF BINARY
    while (remainingChars > 0) {

        if ((remainingChars > 1) && (p[0] == 'S' && p[1] == 'T')) {
            _importer.addOption(BTOFFImporter::VERTEXTEXCOORDS);
            p += 2;
            remainingChars -= 2;
        } else if ((remainingChars > 0) && (p[0] == 'C')) {
            _importer.addOption(BTOFFImporter::VERTEXCOLOR);
            ++p;
            --remainingChars;
        } else if ((remainingChars > 0) && (p[0] == 'N')) {
            _importer.addOption(BTOFFImporter::VERTEXNORMAL);
            ++p;
            --remainingChars;
        } else if ((remainingChars > 0) && (p[0] == '3' )) {
            ++p;
            --remainingChars;
        } else if ((remainingChars > 0) && (p[0] == '4' )) {
            /// \todo Implement extended coordinates
            std::cerr << "Error: Extended coordinates are not supported!" << std::endl;
            ifs.close();
            return false;
        } else if ((remainingChars > 0) && (p[0] == 'n' )) {
            /// \todo Implement any space dimension
            std::cerr << "Error: n-dimensional coordinates are not supported!" << std::endl;
            ifs.close();
            return false;
        } else if ((remainingChars >= 3) && (p[0] == 'O' && p[1] == 'F' && p[2] == 'F')) {
            // Skip "OFF " (plus space):
            p += 4;
            remainingChars -= 4;
        } else if ((remainingChars >= 6) && (strncmp(p, "BINARY", 6) == 0 )) {
            _importer.addOption(BTOFFImporter::BINARY);
            p += 6;
            remainingChars -= 6;
        } else if ( ( remainingChars > 0 ) && ( p[0] == '#' ) ) {
            // Skip the rest of the line since it's a comment
            remainingChars = 0;
        } else {
            // Skip unknown character or space
            ++p;
            --remainingChars;
        }
    }

    // Now extract data type by iterating over
    // the face valences

    unsigned int nV, nF, dummy_uint;
    unsigned int vertexCount = 0;
    unsigned int tmp_count = 0;
    std::string trash;
    std::string str;
    std::istringstream sstr;

    if (_importer.isBinary()) {
        // Parse BINARY file
        float dummy_f;

        // + #Vertices, #Faces, #Edges
        readValue(ifs, nV);
        readValue(ifs, nF);
        readValue(ifs, dummy_uint);

        for (uint i=0; i<nV && !ifs.eof(); ++i) {
            // Skip vertices
            for(int i = 0; i < 3; ++i) readValue(ifs, dummy_f);

            if (_importer.hasVertexNormals()) {
                for(int i = 0; i < 3; ++i) readValue(ifs, dummy_f);
            }

            if (_importer.hasVertexColors()) {
                for(int i = 0; i < 3; ++i) readValue(ifs, dummy_f);
            }

            if (_importer.hasTextureCoords()) {
                for(int i = 0; i < 2; ++i) readValue(ifs, dummy_f);
            }
        }
        for (uint i=0; i<nF; ++i) {
            // Get valence of current face
            readValue(ifs, tmp_count);

			if (ifs.eof())
			  break;

            if (tmp_count > vertexCount)
                vertexCount = tmp_count;

            // Skip the rest

            // Vertex indices
            for (uint i = 0; i < tmp_count; ++i) readValue(ifs, dummy_uint);

            // Get number of color components
            readValue(ifs, tmp_count);

            if (!_importer.hasFaceColors() && tmp_count > 0) {
                _importer.addOption(BTOFFImporter::FACECOLOR);
            }

            // Face color
            for (uint i = 0; i < tmp_count; ++i) {
                readValue(ifs, dummy_f);
            }
        }

    } else {
        // Parse ASCII file

        // Get whole line since there could be comments in it
        getCleanLine(ifs, str);
        sstr.str(str);

        // check if #vertices, #faces and #edges follow
        // on the next line
        if ( str.compare("OFF") == 0 ) {
            getCleanLine(ifs, str);
            sstr.str(str);
        }

        // + #Vertices, #Faces, #Edges
        sstr >> nV;
        sstr >> nF;
        sstr >> dummy_uint;

        // Skip vertices
        for(unsigned int i = 0; i < nV; ++i) {
            getCleanLine(ifs, trash);
        }

        trash = "";

        // Count vertices per face
        for (unsigned int i = 0; i < nF; ++i) {
            sstr.clear();
            getCleanLine(ifs, trash);
            sstr.str(trash);

            sstr >> tmp_count;

            if(tmp_count > vertexCount) vertexCount = tmp_count;

            // Skip vertex indices
            for (uint i = 0; i < tmp_count; ++i) {
                if (sstr.eof()) {
                    emit log(LOGERR,"The OFF File is Malformatted! Aborting...");
                    return false;
                }
                sstr >> dummy_uint;
            }

            // Look if there's at least one face color specified
            // Note: Comments should not be here, so don't treat them
            if (!_importer.hasFaceColors() && !sstr.eof()) {
                _importer.addOption(BTOFFImporter::FACECOLOR);
            }
        }
    }

    ifs.close();

    _importer.maxFaceValence(vertexCount);

    if (vertexCount != 3 || vertexCount == 0 && nF != 0) {
        // Something went wrong
        return false;
    }

    return true;
}

//-----------------------------------------------------------------------------------------------------

bool FileBTOFFPlugin::readOFFFile(QString _filename, BTOFFImporter& _importer)
{
    QFile theFile(_filename);
    if (!theFile.exists()){
        emit log(LOGERR, tr("Unable to load OFF file. File not found!"));
        return false;
    }

    if (!readFileOptions(_filename, _importer)) {
        return false;
    }

    // Let's see if the user has specified some options
    updateUserOptions();

    std::ifstream ifile(
        _filename.toUtf8(),
        (_importer.isBinary() ? std::ios::binary | std::ios::in : std::ios::in)
    );

    unsigned long sz = theFile.size()/1024/1024;
    //the file fits to memory and we still have enough space so pump it in the ram
    if(sz <= 2 * Utils::Memory::queryFreeRAM()) {
        ifile.rdbuf()->pubsetbuf(NULL,theFile.size());
    }

    if (!ifile.is_open() || !ifile.good()) {
        emit log(LOGERR, tr("Cannot open OFF file for reading!"));
        return false;
    }

    assert(ifile);

    return _importer.isBinary() ?
        parseBinary(ifile, _importer, _filename) :
        parseASCII(ifile, _importer, _filename);
}

//-----------------------------------------------------------------------------------------------------

bool FileBTOFFPlugin::parseASCII(
    std::istream& _in,
    BTOFFImporter& _importer,
    QString& _objectName
) {
    unsigned int                idx;
    unsigned int                nV, nF, dummy;
    OpenMesh::Vec3f             v, n;
    OpenMesh::Vec2f             t;
    OpenMesh::Vec3i             c3;
    OpenMesh::Vec3f             c3f;
    OpenMesh::Vec4i             c4;
    OpenMesh::Vec4f             c4f;
    std::vector<VertexHandle>   vhandles;
    FaceHandle                  fh;

    int objectId = -1;
    emit addEmptyObject(DATA_BEZIER_TRIANGLE_MESH, objectId);

    BaseObject* object(0);
    if (!PluginFunctions::getObject(objectId, object)) {
        emit log(LOGERR, tr("Could not create new object!"));
        return false;
    }

    // Set object's name to match file name
    QFileInfo f(_objectName);
    object->setName(f.fileName());

    // Set initial object
    _importer.addObject(object);

    std::string line;
    std::istringstream sstr;

    // read header line
    getCleanLine(_in, line);

    // + #Vertices, #Faces, #Edges
    // Note: We use a stringstream because there
    // could be comments in the line
    getCleanLine(_in, line);
    sstr.str(line);
    sstr >> nV;
    sstr >> nF;
    sstr >> dummy;

    // Reserve memory
    _importer.reserve(nV, nF * _importer.maxFaceValence() /*Upper bound*/, nF);

    // skip empty lines and comments
    std::string tmp;
    while (true) {
        char c = _in.peek();
        if ((c == '\n') || (c == '#')) {
            std::getline(_in, tmp);
        } else {
            break;
        }
    }

    // read vertices: coord [hcoord] [normal] [color] [texcoord]
    for (uint i = 0; i < nV && !_in.eof(); ++i) {
        // Always read VERTEX
        v[0] = getFloat(_in);
        v[1] = getFloat(_in);
        v[2] = getFloat(_in);

        const VertexHandle vh = _importer.addVertex(v);

		//emit log(LOGINFO, tr("num vertices: %1").arg(((BTMeshObject*)object)->mesh()->n_vertices()));

        // perhaps read NORMAL
        if (_importer.hasVertexNormals()){
            n[0] = getFloat(_in);
            n[1] = getFloat(_in);
            n[2] = getFloat(_in);

            if(userReadOptions_ & BTOFFImporter::VERTEXNORMAL) {
                int nid = _importer.addNormal(n);
                _importer.setNormal(vh, nid);
            }
        }

        sstr.clear();
        getCleanLine(_in, line, false);
        sstr.str(line);

        int colorType = getColorType(line, _importer.hasTextureCoords());

        //perhaps read COLOR
        if (_importer.hasVertexColors()){

            std::string trash;

            switch (colorType){
                case 0 : break; //no color
                case 1 : sstr >> trash; break; //one int (isn't handled atm)
                case 2 : sstr >> trash; sstr >> trash; break; //corrupt format (ignore)
                // rgb int
                case 3 :
                sstr >> c3[0];
                sstr >> c3[1];
                sstr >> c3[2];
                if (userReadOptions_ & BTOFFImporter::VERTEXCOLOR) {
                    int cidx = _importer.addColor(OpenMesh::color_cast<BezierTMesh::Color>(c3));
                    _importer.setVertexColor(vh, cidx);
                }
                break;
                // rgba int
                case 4 :
                sstr >> c4[0];
                sstr >> c4[1];
                sstr >> c4[2];
                sstr >> c4[3];
                if (userReadOptions_ & BTOFFImporter::VERTEXCOLOR) {
                    int cidx = _importer.addColor(OpenMesh::color_cast<BezierTMesh::Color>(c4) );
                    _importer.setVertexColor(vh, cidx);
                    _importer.addOption(BTOFFImporter::COLORALPHA);
                }
                break;
                // rgb floats
                case 5 :
                c3f[0] = getFloat(sstr);
                c3f[1] = getFloat(sstr);
                c3f[2] = getFloat(sstr);
                if (userReadOptions_ & BTOFFImporter::VERTEXCOLOR) {
                    int cidx = _importer.addColor(OpenMesh::color_cast<BezierTMesh::Color>(c3f));
                    _importer.setVertexColor(vh, cidx);
                }
                break;
                // rgba floats
                case 6 :
                c4f[0] = getFloat(sstr);
                c4f[1] = getFloat(sstr);
                c4f[2] = getFloat(sstr);
                c4f[3] = getFloat(sstr);
                if (userReadOptions_ & BTOFFImporter::VERTEXCOLOR) {
                    int cidx = _importer.addColor(OpenMesh::color_cast<BezierTMesh::Color>(c4f));
                    _importer.setVertexColor(vh, cidx);
                    _importer.addOption(BTOFFImporter::COLORALPHA);
                }
                break;

                default:
                std::cerr << "Error in file format (colorType = " << colorType << ")\n";
                break;
            }
        }

        //perhaps read TEXTURE COORDS
        if (_importer.hasTextureCoords()) {
            t[0] = getFloat(sstr);
            t[1] = getFloat(sstr);
            if (userReadOptions_ & BTOFFImporter::VERTEXTEXCOORDS) {
                int tcidx = _importer.addTexCoord(t);
                _importer.setVertexTexCoord(vh, tcidx);
            }
        }
    }

    // skip empty lines and comments
    while (true) {
        char c = _in.peek();
        if ((c == '\n') || (c == '#'))
            std::getline(_in, tmp);
        else
            break;
    }

    // faces
    // #N <v1> <v2> .. <v(n-1)> [color spec]
    for (uint i = 0; i < nF; ++i) {
        // nV = number of Vertices for current face
        _in >> nV;

        // If number of faces < 3, we have a degenerated face
        // which we don't allow and thus skip
        if (nV < 3) {
            // Read the rest of the line and dump it
            getCleanLine(_in, line, false);
            // Proceed reading
            continue;
        }

        vhandles.clear();
        for (uint i=0; i<nV; ++i) {
            _in >> idx;
            vhandles.push_back(VertexHandle(idx));
        }

        bool checkManifold = true;
        if (!OpenFlipper::Options::nogui() && loadCheckManifold_ != 0) {
            checkManifold = loadCheckManifold_->isChecked();
        }

        // Check for degenerate faces if specified in gui
        if (checkManifold) {
            if(checkDegenerateFace(vhandles)) {
                fh = _importer.addFace(vhandles);
            } else {
                continue;
            }
        } else {
            fh = _importer.addFace(vhandles);
        }

        //perhaps read face COLOR
        if ( _importer.hasFaceColors() ){

            //take the rest of the line and check how colors are defined
            sstr.clear();
            getCleanLine(_in, line, false);
            sstr.str(line);

            int colorType = getColorType(line, false);

            std::string trash;

            switch (colorType){
                case 0 : break; //no color
                case 1 : sstr >> trash; break; //one int (isn't handled atm)
                case 2 : sstr >> trash; sstr >> trash; break; //corrupt format (ignore)
                    // rgb int
                case 3 :
                  sstr >> c3[0];
                  sstr >> c3[1];
                  sstr >> c3[2];
                if (userReadOptions_ & BTOFFImporter::FACECOLOR) {
                    int cidx = _importer.addColor(OpenMesh::color_cast<BezierTMesh::Color>(c3));
                    _importer.setFaceColor(fh, cidx);
                }
                break;
                // rgba int
                case 4 :
                sstr >> c4[0];
                sstr >> c4[1];
                sstr >> c4[2];
                sstr >> c4[3];
                if (userReadOptions_ & BTOFFImporter::FACECOLOR) {
                    int cidx = _importer.addColor(OpenMesh::color_cast<BezierTMesh::Color>(c4));
                    _importer.setFaceColor(fh, cidx);
                    _importer.addOption(BTOFFImporter::COLORALPHA);
                }
                break;
                // rgb floats
                case 5 :
                c3f[0] = getFloat(sstr);
                c3f[1] = getFloat(sstr);
                c3f[2] = getFloat(sstr);
                if (userReadOptions_ & BTOFFImporter::FACECOLOR) {
                    int cidx = _importer.addColor(OpenMesh::color_cast<BezierTMesh::Color>(c3f));
                    _importer.setFaceColor(fh, cidx);
                }
                break;
                // rgba floats
                case 6 :
                c4f[0] = getFloat(sstr);
                c4f[1] = getFloat(sstr);
                c4f[2] = getFloat(sstr);
                c4f[3] = getFloat(sstr);
                if (userReadOptions_ & BTOFFImporter::FACECOLOR) {
                    int cidx = _importer.addColor(OpenMesh::color_cast<BezierTMesh::Color>(c4f));
                    _importer.setFaceColor(fh, cidx);
                    _importer.addOption(BTOFFImporter::COLORALPHA);
                }
                break;

                default:
                    std::cerr << "Error in file format (colorType = " << colorType << ")\n";
                    break;
            }
        }
    }
    // File was successfully parsed.
    return true;
}

//-----------------------------------------------------------------------------------------------------

bool FileBTOFFPlugin::checkDegenerateFace(const std::vector<VertexHandle>& _v)
{
    int size = _v.size();
    // Check if at least two elements in the list have the same value
    for (int i = 0; i < size; ++i) {
        for (int j = i+1; j < size; ++j) {
            return false;
        }
    }
    return true;
}

//-----------------------------------------------------------------------------------------------------

int FileBTOFFPlugin::getColorType(std::string& _line, bool _texCoordsAvailable)
{
    /*
    0 : no Color
    1 : one int (e.g colormap index)
    2 : two items (error!)
    3 : 3 ints
    4 : 4 ints
    5 : 3 floats
    6 : 4 floats
    */

    // Check if we have any additional information here
    if ( _line.size() < 1 )
        return 0;

    //first remove spaces at start/end of the line
    trimString(_line);

    //count the remaining items in the line
    size_t found;
    int count = 0;

    found = _line.find_first_of(" ");
    while (found != std::string::npos) {
        count++;
        found=_line.find_first_of(" ",found+1);
    }

    if (!_line.empty()) count++;

    if (_texCoordsAvailable) count -= 2;

    if (count == 3 || count == 4) {
        //get first item
        found = _line.find(" ");
        std::string c1 = _line.substr (0,found);

        if (c1.find(".") != std::string::npos){
            count = count == 3 ? 5 : 6;
        }
    }
    return count;
}

//-----------------------------------------------------------------------------------------------------

bool FileBTOFFPlugin::parseBinary(
    std::istream& _in,
    BTOFFImporter& _importer,
    QString& _objectName
) {
    unsigned int                idx;
    unsigned int                nV, nF, dummy;
    float                       dummy_f;
    OpenMesh::Vec3f             v, n;
    OpenMesh::Vec4f             c;
    float                       alpha = 1.0f;
    OpenMesh::Vec2f             t;
    std::vector<VertexHandle>   vhandles;
    FaceHandle                  fh;

    int objectId = -1;
    emit addEmptyObject(DATA_BEZIER_TRIANGLE_MESH, objectId);

    BaseObject* object(0);
    if (!PluginFunctions::getObject( objectId, object )) {
        emit log(LOGERR, tr("Could not create new object!"));
        return false;
    }

    // Set object's name to match file name
    QFileInfo f(_objectName);
    object->setName(f.fileName());

    // Set initial object
    _importer.addObject(object);

    // read header line
    std::string header;
    getCleanLine(_in,header);

    // + #Vertices, #Faces, #Edges
    readValue(_in, nV);
    readValue(_in, nF);
    readValue(_in, dummy);

    // Reserve memory
    _importer.reserve(nV, nF * _importer.maxFaceValence() /*Upper bound*/, nF);

    // read vertices: coord [hcoord] [normal] [color] [texcoord]
    for (uint i=0; i < nV && !_in.eof(); ++i) {
        // Always read Vertex
        readValue(_in, v[0]);
        readValue(_in, v[1]);
        readValue(_in, v[2]);

        const VertexHandle vh = _importer.addVertex(v);

        if ( _importer.hasVertexNormals() ) {
            readValue(_in, n[0]);
            readValue(_in, n[1]);
            readValue(_in, n[2]);

            if ( userReadOptions_ & BTOFFImporter::VERTEXNORMAL ) {
                int nidx = _importer.addNormal(n);
                _importer.setNormal(vh, nidx);
            }
        }

        if ( _importer.hasVertexColors() ) {
            // Vertex colors are always without alpha
            readValue(_in, c[0]);
            readValue(_in, c[1]);
            readValue(_in, c[2]);
            c[3] = 1.0;

            if ( userReadOptions_ & BTOFFImporter::VERTEXCOLOR ) {
                int cidx = _importer.addColor( c );
                _importer.setVertexColor(vh, cidx);
            }
        }

        if ( _importer.hasTextureCoords() ) {
            readValue(_in, t[0]);
            readValue(_in, t[1]);

            if ( userReadOptions_ & BTOFFImporter::VERTEXTEXCOORDS ) {
                int tcidx = _importer.addTexCoord(t);
                _importer.setVertexTexCoord(vh, tcidx);
            }
        }
    }

    int pos = 0;
    int nB = 0;

    // faces
    // #N <v1> <v2> .. <v(n-1)> [color spec]
    for (uint i = 0; i < nF && !_in.eof(); ++i) {
        // Get bytes to be read from this point on
        if (i == 0) {
            pos = _in.tellg();
            _in.seekg(0, std::ios::end);
            nB = _in.tellg();
            nB -= pos;
            _in.seekg(pos);
            // nB now holds the total number of bytes to be read
        }

        readValue(_in, nV);

        // Now that we have the initial face valence
        // we check, if there could possibly be colors
        // after the face specs by checking if
        // the bytes to be read from this point on (nB)
        // equal (nF + nF*nV)*4 (each line of [nV V_1 ... V..nV]).
        // If not, we have more bytes to be read than
        // actual face definitions.
        // So if we have at least nF additional bytes
        // we can read the number of color components after each face
        // definition.
        if (i == 0) {
            // Always make sure that we only deal with
            // integers and floats/doubles
            if (nB % 4 == 0) {
                // Cut down number of bytes to number
                // of elements to be read
                nB /= 4;
                nB -= nF + nF*nV;

                // We don't have additional color components
                // Case nB < 0: Face valence is not constant
                // throughout the mesh
                readColorComp_ = nB <= 0 ? false : extendedFaceColorTest(_in, nV, nF, nB);
                // Not enough additional elements to read
                // This should actually never happen...
                // or nB >= nF -> perform extended
                // face color component test
            }
        }

        // Check if the face has at least valence 3
        // if not, skip the current face
        if (nV < 3) {
            // Read in following vertex indices and dump them
            for (uint j = 0; j < nV; ++j) {
                readValue(_in, dummy);
            }
            // Read in color components if available
            // and dump them
            if (readColorComp_) {
                // Number of color components
                readValue(_in, nV);
                for (uint j = 0; j < nV; ++j) {
                    readValue(_in, dummy_f);
                }
            }
            // Proceed reading
            continue;
        }

        // Read vertex indices of current face
        vhandles.clear();
        for (uint j = 0; j < nV; ++j) {
            readValue(_in, idx);
            vhandles.push_back(VertexHandle(idx));
        }

        fh = _importer.addFace(vhandles);

        if ( !readColorComp_ ) {
            // Some binary files that were created via an OFF writer
            // that doesn't comply with the OFF specification
            // don't specify the number of color components before
            // the face specs.
            nV = 0;
        } else {
            // nV now holds the number of color components
            readValue(_in, nV);
        }

        // valid face color:
        if ( nV == 3 || nV == 4 ) {
            // Read standard rgb color
            for (uint k = 0; k < 3; ++k) {
                readValue(_in, c[k]);
                --nV;
            }
            // Color has additional alpha value
            if (nV == 1) {
                readValue(_in, alpha);
            }

            if (userReadOptions_ & BTOFFImporter::FACECOLOR) {
                if (userReadOptions_ & BTOFFImporter::COLORALPHA) {
                    int cidx = _importer.addColor(OpenMesh::Vec4f(c[0], c[1], c[2], alpha));
                    _importer.setFaceColor( fh,  cidx );
                    _importer.addOption(BTOFFImporter::COLORALPHA);
                } else {
                    int cidx = _importer.addColor(OpenMesh::color_cast<OpenMesh::Vec4f>(c));
                    _importer.setFaceColor( fh,  cidx );
                }
            }
        } else {
            // Skip face colors since they are not in a supported format
            for (uint i = 0; i < nV; ++i) {
                readValue(_in, dummy_f);
            }
        }
    }
    // File was successfully parsed.
    return true;
}

//-----------------------------------------------------------------------------------------------------

bool FileBTOFFPlugin::extendedFaceColorTest(std::istream& _in, uint _nV, uint _nF, int _nB) const
{
    // Perform the extended and even more reliable color
    // component test. Read an integer n (starting with the face
    // valence) and skip n*4 bytes. Repeat this nF times.
    // After this we have two cases:
    //
    // Case 1: The file contains face color components
    // and we interpreted the number of face components as face
    // valence which results in a bunch of bytes that still are to be read
    // after nF cycles.
    //
    // Case 2: The mesh has varying face valences and has
    // therefor been wrongly detected as containing face color components.
    // If this has happened, the following test will result in
    // no bytes to be left for reading after nF reading cycles.

    uint nV = _nV;
    uint dummy = 0;

    // Get current file pointer
    int pos = _in.tellg();

    for (uint k = 0; k < _nF; ++k) {
        // Remember: The first nV has already been read
        if (k != 0) readValue(_in, nV);
        // Skip the following nV values
        for (uint z = 0; z < nV; ++z) {
            readValue(_in, dummy);
        }
    }

    // Get position after all the reading has been done
    int currPos = _in.tellg();

    // Reset read pointer to where we were
    _in.seekg(pos);

    return _nB - currPos != 0;
}

//-----------------------------------------------------------------------------------------------------

int FileBTOFFPlugin::loadObject(QString _filename)
{
    BTOFFImporter importer;
    // Parse file
    readOFFFile( _filename, importer );

    // Finish importing
    importer.finish();

    BaseObject* object = importer.getObject();

    if (!object) {
        return -1;
    }

    object->setFromFileName(_filename);

	// Handle new BezierTriangleMeshes
	BTMeshObject* btMeshObj = dynamic_cast<BTMeshObject*> (object);

	if (btMeshObj) {
		if (!importer.hasVertexNormals() || (userReadOptions_ & BTOFFImporter::FORCE_NONORMALS)) {
			emit log(LOGINFO, tr("loadObject: Computing vertex and face normals."));
			btMeshObj->mesh()->update_normals();
		}
		else {
			emit log(LOGINFO, tr("loadObject: Computing face normals."));
			btMeshObj->mesh()->update_face_normals();
		}
		backupTextureCoordinates(*(btMeshObj->mesh()));
	}

	btMeshObj->mesh()->setRenderable();

    //general stuff
    emit updatedObject(object->id(), UPDATE_ALL);
    emit openedFile(object->id());

    return object->id();
}

//-----------------------------------------------------------------------------------------------------

bool FileBTOFFPlugin::saveObject(int _id, QString _filename)
{
    BaseObjectData* object;
    if (!PluginFunctions::getObject(_id,object)) {
        emit log(
            LOGERR,
            tr("saveObject : cannot get object id %1 for save name %2").arg(_id).arg(_filename)
        );
        return false;
    }

    std::string filename = std::string( _filename.toUtf8());

    bool binary = userWriteOptions_ & BTOFFImporter::BINARY;
    std::fstream ofs(
        filename.c_str(),
        (binary ? std::ios_base::out | std::ios_base::binary : std::ios_base::out)
    );

    if (!ofs) {
        emit log(LOGERR, tr("saveObject : Cannot not open file %1 for writing!").arg(_filename));
        return false;
    }

    // Get user specified options
    updateUserOptions();

    object->setFromFileName(_filename);
    object->setName(object->filename());

    BTMeshObject* btMeshObj = dynamic_cast<BTMeshObject*>(object);

    if (writeMesh(ofs, *btMeshObj->mesh(), *btMeshObj)) {
        emit log(LOGINFO, tr("Saved object to ") + _filename);
        ofs.close();
        return true;
    } else {
        emit log(LOGERR, tr("Unable to save ") + _filename);
        ofs.close();
        return false;
    }
}

//-----------------------------------------------------------------------------------------------------

template <class MeshT>
void FileBTOFFPlugin::backupTextureCoordinates(MeshT& _mesh)
{
    // Create a backup of the original per Vertex texture Coordinates
    if (_mesh.has_vertex_texcoords2D()) {
        OpenMesh::VPropHandleT<typename MeshT::TexCoord2D> oldVertexCoords;
        if (!_mesh.get_property_handle(oldVertexCoords, "Original Per Vertex Texture Coords"))
            _mesh.add_property(oldVertexCoords, "Original Per Vertex Texture Coords");

        for (typename MeshT::VertexIter v_it = _mesh.vertices_begin();
            v_it != _mesh.vertices_end(); ++v_it) {
            _mesh.property(oldVertexCoords, *v_it) = _mesh.texcoord2D(*v_it);
        }
    }

    // Create a backup of the original per Face texture Coordinates
    if (_mesh.has_halfedge_texcoords2D()) {
        OpenMesh::HPropHandleT<typename MeshT::TexCoord2D> oldHalfedgeCoords;
        if (!_mesh.get_property_handle(oldHalfedgeCoords,"Original Per Face Texture Coords"))
            _mesh.add_property(oldHalfedgeCoords,"Original Per Face Texture Coords");

        for (typename MeshT::HalfedgeIter he_it = _mesh.halfedges_begin();
            he_it != _mesh.halfedges_end(); ++he_it) {
            _mesh.property(oldHalfedgeCoords, *he_it) = _mesh.texcoord2D(*he_it);
        }
    }
}

//-----------------------------------------------------------------------------------------------------

QWidget* FileBTOFFPlugin::saveOptionsWidget(QString /*_currentFilter*/)
{
    if (saveOptions_ == 0) {
        //generate widget
        saveOptions_ = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout();
        layout->setAlignment(Qt::AlignTop);

        saveBinary_ = new QCheckBox("Save Binary");
        layout->addWidget(saveBinary_);

        saveVertexColor_ = new QCheckBox("Save Vertex Colors");
        layout->addWidget(saveVertexColor_);

        saveFaceColor_ = new QCheckBox("Save Face Colors");
        layout->addWidget(saveFaceColor_);

        saveAlpha_ = new QCheckBox("Save Color Alpha");
        layout->addWidget(saveAlpha_);

        saveNormals_ = new QCheckBox("Save Normals");
        layout->addWidget(saveNormals_);

        saveTexCoords_ = new QCheckBox("Save TexCoords");
        layout->addWidget(saveTexCoords_);

        savePrecisionLabel_ = new QLabel("Writer Precision");
        layout->addWidget(savePrecisionLabel_);

        savePrecision_ = new QSpinBox();
        savePrecision_->setMinimum(1);
        savePrecision_->setMaximum(12);
        savePrecision_->setValue(6);
        layout->addWidget(savePrecision_);

        saveDefaultButton_ = new QPushButton("Make Default");
        layout->addWidget(saveDefaultButton_);

        saveOptions_->setLayout(layout);

        connect(saveBinary_, SIGNAL(clicked(bool)), savePrecision_, SLOT(setDisabled(bool)));
        connect(saveDefaultButton_, SIGNAL(clicked()), this, SLOT(slotSaveDefault()));

        saveBinary_->setChecked(OpenFlipperSettings().value("FileBTOff/Save/Binary",false).toBool());
        saveVertexColor_->setChecked(OpenFlipperSettings().value("FileBTOff/Save/VertexColor",true).toBool());
        saveFaceColor_->setChecked(OpenFlipperSettings().value("FileBTOff/Save/FaceColor",true).toBool());
        saveAlpha_->setChecked(OpenFlipperSettings().value("FileBTOff/Save/Alpha",true).toBool());
        saveNormals_->setChecked(OpenFlipperSettings().value("FileBTOff/Save/Normals",true).toBool());
        saveTexCoords_->setChecked(OpenFlipperSettings().value("FileBTOff/Save/TexCoords",true).toBool());
    }
    return saveOptions_;
}

//-----------------------------------------------------------------------------------------------------

QWidget* FileBTOFFPlugin::loadOptionsWidget(QString /*_currentFilter*/)
{
    if (loadOptions_ == 0) {
        //generate widget
        loadOptions_ = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout();
        layout->setAlignment(Qt::AlignTop);

        loadVertexColor_ = new QCheckBox("Load Vertex Colors");
        layout->addWidget(loadVertexColor_);

        loadFaceColor_ = new QCheckBox("Load Face Colors");
        layout->addWidget(loadFaceColor_);

        loadAlpha_ = new QCheckBox("Load Color Alpha");
        layout->addWidget(loadAlpha_);

        loadNormals_ = new QCheckBox("Load Normals");
        layout->addWidget(loadNormals_);

        loadTexCoords_ = new QCheckBox("Load TexCoords");
        layout->addWidget(loadTexCoords_);

        loadCheckManifold_ = new QCheckBox("Check for manifold configurations");
        layout->addWidget(loadCheckManifold_);

        loadDefaultButton_ = new QPushButton("Make Default");
        layout->addWidget(loadDefaultButton_);

        loadOptions_->setLayout(layout);

        connect(loadDefaultButton_, SIGNAL(clicked()), this, SLOT(slotLoadDefault()));

        loadVertexColor_->setChecked( OpenFlipperSettings().value("FileBTOff/Load/VertexColor",true).toBool() );
        loadFaceColor_->setChecked( OpenFlipperSettings().value("FileBTOff/Load/FaceColor",true).toBool()  );
        loadAlpha_->setChecked( OpenFlipperSettings().value("FileBTOff/Load/Alpha",true).toBool()  );
        loadNormals_->setChecked( OpenFlipperSettings().value("FileBTOff/Load/Normals",true).toBool()  );
        loadTexCoords_->setChecked( OpenFlipperSettings().value("FileBTOff/Load/TexCoords",true).toBool()  );
    }

    return loadOptions_;
}

void FileBTOFFPlugin::slotLoadDefault()
{
    OpenFlipperSettings().setValue("FileBTOff/Load/VertexColor", loadVertexColor_->isChecked());
    OpenFlipperSettings().setValue("FileBTOff/Load/FaceColor", loadFaceColor_->isChecked());
    OpenFlipperSettings().setValue("FileBTOff/Load/Alpha", loadAlpha_->isChecked());
    OpenFlipperSettings().setValue("FileBTOff/Load/Normals", loadNormals_->isChecked());
    OpenFlipperSettings().setValue("FileBTOff/Load/TexCoords", loadTexCoords_->isChecked());

    OpenFlipperSettings().setValue("Core/File/UseLoadDefaults", true);
}

void FileBTOFFPlugin::slotSaveDefault()
{
    OpenFlipperSettings().setValue("FileBTOff/Save/Binary", saveBinary_->isChecked());
    OpenFlipperSettings().setValue("FileBTOff/Save/VertexColor", saveVertexColor_->isChecked());
    OpenFlipperSettings().setValue("FileBTOff/Save/FaceColor", saveFaceColor_->isChecked());
    OpenFlipperSettings().setValue("FileBTOff/Save/Alpha", saveAlpha_->isChecked());
    OpenFlipperSettings().setValue("FileBTOff/Save/Normals", saveNormals_->isChecked());
    OpenFlipperSettings().setValue("FileBTOff/Save/TexCoords", saveTexCoords_->isChecked());
}
