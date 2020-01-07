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

#include "FileBTM.hh"

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
FileBTMPlugin::FileBTMPlugin() :
    loadOptions_(0),
    saveOptions_(0),
    saveBinary_(0),
    savePrecisionLabel_(0),
    savePrecision_(0),
    saveDefaultButton_(0),
    loadCheckManifold_(0),
    loadDefaultButton_(0),
    userReadOptions_(0),
    userWriteOptions_(0),
    trimeshOptions_(BTMImporter::NONE) {}

//-----------------------------------------------------------------------------------------------------

void FileBTMPlugin::initializePlugin() {
    // initialize standard options that can then be changed in the file dialogs
    if (OpenFlipperSettings().value("FileBTM/Save/Binary",true).toBool())
        userWriteOptions_ |= BTMImporter::BINARY;
}

//-----------------------------------------------------------------------------------------------------

QString FileBTMPlugin::getLoadFilters()
{
    return QString(tr("BezierTriangleMesh File Format files ( *.btm )"));
};

//-----------------------------------------------------------------------------------------------------

QString FileBTMPlugin::getSaveFilters()
{
    return QString(tr("BezierTriangleMesh File Format files ( *.btm )"));
};

//-----------------------------------------------------------------------------------------------------

DataType FileBTMPlugin::supportedType()
{
    return DATA_BEZIER_TRIANGLE_MESH;
}

//-----------------------------------------------------------------------------------------------------

void FileBTMPlugin::trimString(std::string& _string)
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

bool FileBTMPlugin::getCleanLine( std::istream& ifs , std::string& _string, bool _skipEmptyLines)
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

void FileBTMPlugin::updateUserOptions()
{
    // If the options dialog has not been initialized, keep
    // the initial values

    if (OpenFlipper::Options::nogui() )
        return;

    // save options
    if (saveBinary_) {
        if (saveBinary_->isChecked())
            userWriteOptions_ |= BTMImporter::BINARY;
        else if(userWriteOptions_ & BTMImporter::BINARY)
            userWriteOptions_ -= BTMImporter::BINARY;
    }
}

//-----------------------------------------------------------------------------------------------------

bool FileBTMPlugin::readFileOptions(QString _filename, BTMImporter& _importer)
{
    /* Constitution of a BTM-file
       ==================================================================
       BTM [BINARY] # comment
       degree nV nF nE # number of vertices, faces and edges (edges are skipped)
       v[0] v[1] v[2]
       ...
       faceValence vIdx[0] ... vIdx[faceValence-1] cp0[0] cp0[1] cp0[2] ... cpn[0] cpn[1] cpn[2]
       ...
       ==================================================================
    */

    const unsigned int LINE_LEN = 4096;

    std::ifstream ifs(_filename.toUtf8(), std::ios_base::binary);

    if ((!ifs.is_open()) || (!ifs.good())) {
        emit log(LOGERR, tr("Error: Could not read file options of BTM-file! Aborting."));
        return false;
    }

    // read 1st line
    char line[LINE_LEN], *p;
    ifs.getline(line, LINE_LEN);
    p = line;

    int remainingChars = ifs.gcount();

    // check header: BTM [BINARY]
    while (remainingChars > 0) {

        if ((remainingChars >= 3) && (p[0] == 'O' && p[1] == 'F' && p[2] == 'F')) {
            // Skip "BTM " (plus space):
            p += 4;
            remainingChars -= 4;
        } else if ((remainingChars >= 6) && (strncmp(p, "BINARY", 6) == 0)) {
            _importer.addOption(BTMImporter::BINARY);
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

    // now extract data by iterating over
    // the face valences

    unsigned int deg, nV, nF, dummy_uint, cpNum;
    unsigned int vertexCount = 0;
    unsigned int tmp_count = 0;
    std::string trash;
    std::string str;
    std::istringstream sstr;

    if (_importer.isBinary()) {
        // parse BINARY file
        float dummy_f;

        // degree #vertices, #faces, #edges
        readValue(ifs, deg);
        readValue(ifs, nV);
        readValue(ifs, nF);
        readValue(ifs, dummy_uint);
    } else {
        // parse ASCII file

        // Get whole line since there could be comments in it
        getCleanLine(ifs, str);
        sstr.str(str);

        // check if degree, #vertices, #faces and #edges follow
        // on the next line
        if (str.compare("BTM") == 0) {
            getCleanLine(ifs, str);
            sstr.str(str);
        }

        // degree, #vertices, #faces, #edges
        sstr >> deg;
        sstr >> nV;
        sstr >> nF;
        sstr >> dummy_uint;

        // skip vertices
        for(unsigned int i = 0; i < nV; ++i) {
            getCleanLine(ifs, trash);
        }

        trash = "";

        // count vertices per face
        for (unsigned int i = 0; i < nF; ++i) {
            sstr.clear();
            getCleanLine(ifs, trash);
            sstr.str(trash);

            sstr >> tmp_count;

            if (tmp_count > vertexCount) vertexCount = tmp_count;

            // skip vertex indices
            for (uint i = 0; i < tmp_count; ++i) {
                if (sstr.eof()) {
                    emit log(LOGERR,"The BTM File is Malformatted! Aborting...");
                    return false;
                }
                sstr >> dummy_uint;
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

bool FileBTMPlugin::readOFFFile(QString _filename, BTMImporter& _importer)
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
    // the file fits to memory and we still have enough space so pump it in the ram
    if(sz <= 2 * Utils::Memory::queryFreeRAM()) {
        ifile.rdbuf()->pubsetbuf(NULL,theFile.size());
    }

    if (!ifile.is_open() || !ifile.good()) {
        emit log(LOGERR, tr("Cannot open BTM file for reading!"));
        return false;
    }

    assert(ifile);

    return _importer.isBinary() ?
        parseBinary(ifile, _importer, _filename) :
        parseASCII(ifile, _importer, _filename);
}

//-----------------------------------------------------------------------------------------------------

bool FileBTMPlugin::parseASCII(
    std::istream& _in,
    BTMImporter& _importer,
    QString& _objectName
) {
    unsigned int                idx;
    unsigned int                degree, nV, nF, dummy;
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

    std::string line;
    std::istringstream sstr;

    // read header line
    getCleanLine(_in, line);

    // degree, #vertices, #faces, #edges
    // Note: We use a stringstream because there
    // could be comments in the line
    getCleanLine(_in, line);
    sstr.str(line);
    sstr >> degree;
    sstr >> nV;
    sstr >> nF;
    sstr >> dummy;

	// Set initial object
	_importer.addObject(object, degree);

    // reserve memory
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

    // read vertices: coord
    for (uint i = 0; i < nV && !_in.eof(); ++i) {
        // Always read VERTEX
        v[0] = getFloat(_in);
        v[1] = getFloat(_in);
        v[2] = getFloat(_in);

        const VertexHandle vh = _importer.addVertex(v);

        sstr.clear();
        getCleanLine(_in, line, false);
        sstr.str(line);
    }

    // skip empty lines and comments
    while (true) {
        char c = _in.peek();
        if ((c == '\n') || (c == '#'))
            std::getline(_in, tmp);
        else
            break;
    }

    uint cpPerFace = (degree+1)*(degree+2) / 2;

    // control point values
    float cpx, cpy, cpz;

    // faces
    // #valence <v1> <v2> .. <v(n-1)> <cp0[0]> <cp0[1]> <cp0[2]> .. <cpn[0]> <cpn[1]> <cpn[2]>
    for (uint i = 0; i < nF; ++i) {
        // nV = number of vertices for current face
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
        for (uint i = 0; i < nV; ++i) {
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

		// invalid face ?
		if (fh < 0) {
			std::cerr << "invalid face handle (complex edge?)\n";
			continue;
		}

		BezierTMesh::FaceHandle fHandle = _importer.face(fh);
		// read control points
		for (uint j = 0; j < cpPerFace; ++j) {
			_in >> cpx;
			_in >> cpy;
			_in >> cpz;
			_importer.addControlPoint(fHandle, cpx, cpy, cpz);
		}
    }

    // file was successfully parsed.
    return true;
}

//-----------------------------------------------------------------------------------------------------

bool FileBTMPlugin::checkDegenerateFace(const std::vector<VertexHandle>& _v)
{
    int size = _v.size();
    // check if at least two elements in the list have the same value
    for (int i = 0; i < size; ++i) {
        for (int j = i+1; j < size; ++j) {
            return false;
        }
    }
    return true;
}

//-----------------------------------------------------------------------------------------------------

bool FileBTMPlugin::parseBinary(
    std::istream& _in,
    BTMImporter& _importer,
    QString& _objectName
) {
    unsigned int                idx;
    unsigned int                degree, nV, nF, dummy;
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

    // read header line
    std::string header;
    getCleanLine(_in,header);

    // degree, #vertices, #faces, #edges
    readValue(_in, degree);
    readValue(_in, nV);
    readValue(_in, nF);
    readValue(_in, dummy);

	// Set initial object
	_importer.addObject(object, degree);

    // reserve memory
    _importer.reserve(nV, nF * _importer.maxFaceValence() /*Upper bound*/, nF);

    // read vertices: coord
    for (uint i=0; i < nV && !_in.eof(); ++i) {

        readValue(_in, v[0]);
        readValue(_in, v[1]);
        readValue(_in, v[2]);

         _importer.addVertex(v);
    }

    int pos = 0;
    int nB = 0;

    uint cpPerFace = (degree+1)*(degree+2) / 2;

    // control point values
    float cpx, cpy, cpz;

    // faces
    // #N <v1> <v2> .. <v(n-1)> <cp0[0]> <cp0[1]> <cp0[2]> .. <cpn[0]> <cpn[1]> <cpn[2]>
    for (uint i = 0; i < nF && !_in.eof(); ++i) {
        // get bytes to be read from this point on
        if (i == 0) {
            pos = _in.tellg();
            _in.seekg(0, std::ios::end);
            nB = _in.tellg();
            nB -= pos;
            _in.seekg(pos);
            // nB now holds the total number of bytes to be read
        }

        readValue(_in, nV);

        // check if the face has at least valence 3
        // if not, skip the current face
        if (nV < 3) {
            // read in following vertex indices and dump them
            for (uint j = 0; j < nV; ++j) {
                readValue(_in, dummy);
            }
            // Read in color components if available
            // and dump them
            float d_dummy;
            for (uint j = 0; j < cpPerFace; ++j) {
                readValue(_in, d_dummy);
                readValue(_in, d_dummy);
                readValue(_in, d_dummy);
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

        nV = 0;

		if (fh < 0) {
			std::cerr << "invalid face handle (complex edge?)\n";
			continue;
		}

		BezierTMesh::FaceHandle fHandle = _importer.face(fh);
        // read control points
        for (uint j = 0; j < cpPerFace; ++j) {
            _in >> cpx;
            _in >> cpy;
            _in >> cpz;
            _importer.addControlPoint(fHandle, cpx, cpy, cpz);
        }
    }
    // File was successfully parsed.
    return true;
}

//-----------------------------------------------------------------------------------------------------

int FileBTMPlugin::loadObject(QString _filename)
{
    BTMImporter importer;
    // Parse file
    readOFFFile(_filename, importer);

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
		emit log(LOGINFO, tr("loadObject: Computing vertex and face normals."));
		btMeshObj->mesh()->update_normals();
	}

	btMeshObj->mesh()->setRenderable();

    //general stuff
    emit updatedObject(object->id(), UPDATE_ALL);
    emit openedFile(object->id());

    return object->id();
}

//-----------------------------------------------------------------------------------------------------

bool FileBTMPlugin::saveObject(int _id, QString _filename)
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

    bool binary = userWriteOptions_ & BTMImporter::BINARY;
    std::fstream ofs(
        filename.c_str(),
        (binary ? std::ios_base::out | std::ios_base::binary : std::ios_base::out)
    );

    if (!ofs) {
        emit log(LOGERR, tr("saveObject : cannot not open file %1 for writing!").arg(_filename));
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

QWidget* FileBTMPlugin::saveOptionsWidget(QString /*_currentFilter*/)
{
    if (saveOptions_ == 0) {
        //generate widget
        saveOptions_ = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout();
        layout->setAlignment(Qt::AlignTop);

        saveBinary_ = new QCheckBox("Save Binary");
        layout->addWidget(saveBinary_);

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
    }
    return saveOptions_;
}

//-----------------------------------------------------------------------------------------------------

QWidget* FileBTMPlugin::loadOptionsWidget(QString /*_currentFilter*/)
{
    if (loadOptions_ == 0) {
        //generate widget
        loadOptions_ = new QWidget();
        QVBoxLayout* layout = new QVBoxLayout();
        layout->setAlignment(Qt::AlignTop);

        loadCheckManifold_ = new QCheckBox("Check for manifold configurations");
        layout->addWidget(loadCheckManifold_);

        loadDefaultButton_ = new QPushButton("Make Default");
        layout->addWidget(loadDefaultButton_);

        loadOptions_->setLayout(layout);

        connect(loadDefaultButton_, SIGNAL(clicked()), this, SLOT(slotLoadDefault()));
    }

    return loadOptions_;
}

void FileBTMPlugin::slotLoadDefault()
{
    OpenFlipperSettings().setValue("Core/File/UseLoadDefaults", true);
}

void FileBTMPlugin::slotSaveDefault()
{
    OpenFlipperSettings().setValue("FileBTM/Save/Binary", saveBinary_->isChecked());
}

bool FileBTMPlugin::writeBinaryData(std::ostream& _out, BezierTMesh& _mesh )
{
    Vec3f v, n;
    Vec2f t;
    OpenMesh::Vec4f c(1.0,1.0,1.0,1.0);
    OpenMesh::Vec3f p;

    auto vit = _mesh.vertices_begin();
    auto end_vit = _mesh.vertices_end();

    // degree #vertices #faces #edges
    writeValue(_out, (uint)_mesh.degree());
    writeValue(_out, (uint)_mesh.n_vertices());
    writeValue(_out, (uint)_mesh.n_faces());
    writeValue(_out, (uint)_mesh.n_edges());

    // write vertex data
    for (; vit != end_vit; ++vit) {
        // write vertex p[0] p[1] p[2]
        p = _mesh.point(*vit);
        writeValue(_out, p[0]);
        writeValue(_out, p[1]);
        writeValue(_out, p[2]);
    }

    auto fit = _mesh.faces_begin();
    auto end_fit = _mesh.faces_end();
    BezierTMesh::FaceVertexIter fvit;

    // write face data
    for (; fit != end_fit; ++fit) {

        // write face valence
        writeValue(_out, _mesh.valence(*fit));

        // write vertex indices
        for (fvit = _mesh.fv_iter(*fit); fvit.is_valid(); ++fvit) {
            writeValue(_out, fvit->idx());
        }

        // write control points
        auto &cp = _mesh.data(*fit);
        for (auto cpit = cp.cpBegin(); cpit != cp.cpEnd(); ++cpit) {
            p = *cpit;
            writeValue(_out, p[0]);
            writeValue(_out, p[1]);
            writeValue(_out, p[2]);
        }
    }

    return true;
}

bool FileBTMPlugin::writeASCIIData(std::ostream& _out, BezierTMesh& _mesh )
{
    BezierTMesh::Point p;

    auto vit = _mesh.vertices_begin();
    auto end_vit = _mesh.vertices_end();

    // degree #vertices #faces #edges
	_out << _mesh.degree() << " " << _mesh.n_vertices() << " ";
	_out << _mesh.n_faces() << " " << _mesh.n_edges();

    // write vertex data
    for (; vit != end_vit; ++vit) {

        _out << "\n";

        // write vertex position p[0] p[1] p[2]
        p = _mesh.point(*vit);
        _out << p[0] << " " << p[1] << " " << p[2];
    }

    auto fit = _mesh.faces_begin();
    auto end_fit = _mesh.faces_end();

    BezierTMesh::FaceVertexIter fvit;

    // write face data
    for (; fit != end_fit; ++fit) {
        _out << "\n";
        // write face valence
        _out << _mesh.valence(*fit);

        // write vertex indices
        for (fvit = _mesh.fv_iter(*fit); fvit.is_valid(); ++fvit) {
            _out << " " << fvit->idx();
        }

        // write control points
        auto &cp = _mesh.data(*fit);
        for (auto cpit = cp.cpBegin(); cpit != cp.cpEnd(); ++cpit) {
            p = *cpit;
            _out << " " << p[0] << " " << p[1] << " " << p[2];
        }
        _out << "\n";
    }

    return true;
}

bool FileBTMPlugin::writeMesh(std::ostream& _out, BezierTMesh& _mesh, BaseObject &_baseObj)
{
    /*****************
    * HEADER
    ******************/

    // Write
    _out << "BTM";

    // Write BINARY
    if (userWriteOptions_ & BTMImporter::BINARY) {
        _out << " BINARY";
    }

    _out << "\n";

    /*
     * Comment
     */
    OpenMesh::MPropHandleT<std::string> mp_comment;

    if (_baseObj.hasComments()) {
        _out << "# %% BEGIN OPENFLIPPER_COMMENT %%" << std::endl;
        std::istringstream comment(_baseObj.getAllCommentsFlat().toStdString());
        std::string commentLine;
        while (std::getline(comment, commentLine)) {
            _out << "# " << commentLine << std::endl;
        }
        _out << "# %% END OPENFLIPPER_COMMENT %%" << std::endl;
    }

    /*****************
    * DATA
    ******************/

    // Call corresponding write routine
    if(userWriteOptions_ & BTMImporter::BINARY) {
        return writeBinaryData(_out, _mesh);
    } else {
        if (!OpenFlipper::Options::savingSettings() && saveOptions_ != 0)
            _out.precision(savePrecision_->value());

        return writeASCIIData(_out, _mesh);
    }
}
