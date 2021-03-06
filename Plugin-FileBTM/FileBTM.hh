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

#include <QObject>
#include <QCheckBox>
#include <QLabel>
#include <QSpinBox>
#include <QPushButton>
#include <QComboBox>


#include <OpenFlipper/common/Types.hh>
#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/FileInterface.hh>
#include <OpenFlipper/BasePlugin/LoadSaveInterface.hh>
#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/BasePlugin/ScriptInterface.hh>
#include <OpenFlipper/BasePlugin/TypeInterface.hh>
#include <OpenFlipper/BasePlugin/StatusbarInterface.hh>

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

// Binary file support
#include <OpenMesh/Core/IO/SR_store.hh>
#include <limits>

#include "BTMImporter.hh"

class FileBTMPlugin : public QObject,
    BaseInterface,
    FileInterface,
    LoadSaveInterface,
    LoggingInterface,
    ScriptInterface,
    StatusbarInterface
{
    Q_OBJECT
    Q_INTERFACES(FileInterface)
    Q_INTERFACES(LoadSaveInterface)
    Q_INTERFACES(LoggingInterface)
    Q_INTERFACES(BaseInterface)
    Q_INTERFACES(ScriptInterface)
    Q_INTERFACES(StatusbarInterface)
    Q_PLUGIN_METADATA(IID "org.OpenFlipper.Plugins.Plugin-FileBTM")

signals:
    void openedFile(int _id);
    void addEmptyObject(DataType _type, int& _id);
    void load(QString _filename, DataType _type, int& _id);
    void save(int _id , QString _filename);
    void log(Logtype _type, QString _message);
    void log(QString _message);
    void updateView();
    void updatedObject(int _identifier, const UpdateType& _type);

    void deleteObject(int _id);

    // StatusbarInterface
    void showStatusMessage(QString _message, int _timeout = 0);
    void setStatus(ApplicationStatus::applicationStatus _status);

private slots:

    void fileOpened(int /*_id */) {};

    void noguiSupported() {} ;

    void initializePlugin();

    /// Slot called when user wants to save the given Load options as default
    void slotLoadDefault();

    /// Slot called when user wants to save the given Save options as default
    void slotSaveDefault();

public:

    FileBTMPlugin();

    ~FileBTMPlugin() {};

    QString name() { return (QString("FileBTM")); };
    QString description() { return (QString(tr("Load/Save BTM-Files as BezierTriangleMesh"))); };

    DataType supportedType();

    QString getSaveFilters();
    QString getLoadFilters();

    QWidget* saveOptionsWidget(QString /*_currentFilter*/);
    QWidget* loadOptionsWidget(QString /*_currentFilter*/);

public slots:

    /// Loads Object and converts it to a triangle mesh if possible
    int loadObject(QString _filename);

    bool saveObject(int _id, QString _filename);

    QString version() { return QString("1.1"); };

private:

    /// Before Parsing the actual file, read all features supported
    bool readFileOptions(QString _filename, BTMImporter &_importer);

    /// Read OFF file and parse it
    bool readOFFFile(QString _filename, BTMImporter &_importer);

    /// Parse ascii OFF file
    bool parseASCII(std::istream& _in, BTMImporter &_importer, QString &_objectName);

    /// Parse binary OFF file
    bool parseBinary(std::istream& _in, BTMImporter &_importer, QString &_objectName);

    /// Update user options depending on which options have been selected
    /// on the load dialog
    void updateUserOptions();

    // Binary reader and writer helpers
    void readValue(std::istream& _in, float &_value) const
    {
        float tmp;
        OpenMesh::IO::restore(_in , tmp, false); //assuming LSB byte order
        _value = tmp;
    }

    void readValue(std::istream& _in, int &_value) const
    {
        OpenMesh::IO::int32_t tmp;
        OpenMesh::IO::restore( _in , tmp, false ); //assuming LSB byte order
        _value = tmp;
    }

    void readValue(std::istream& _in, unsigned int &_value) const
    {
        OpenMesh::IO::uint32_t tmp;
        OpenMesh::IO::restore( _in , tmp, false ); //assuming LSB byte order
        _value = tmp;
    }

    void writeValue(std::ostream& _out, int value) const
    {
        OpenMesh::IO::uint32_t tmp = value;
        OpenMesh::IO::store(_out, tmp, false);
    }

    void writeValue(std::ostream& _out, unsigned int value) const
    {
        OpenMesh::IO::uint32_t tmp = value;
        OpenMesh::IO::store(_out, tmp, false);
    }

    void writeValue(std::ostream& _out, float value) const
    {
        float tmp = value;
        OpenMesh::IO::store(_out, tmp, false);
    }

    void trimString(std::string& _string);

    /** \brief Function to retrieve next line
     *
     * @param ifs             The input stream we operate on
     * @param _string         This will be the output string
     * @param _skipEmptyLines Skip empty/comment lines? If not, empty strings will be returned if comment or (remaining) line is empty
     * @return
     */
    bool getCleanLine(std::istream& ifs , std::string& _string, bool _skipEmptyLines=true);

    /// Check for degenerate faces before adding them
    bool checkDegenerateFace(const std::vector<VertexHandle>& _v);

    /// Writer function
    bool writeMesh(std::ostream& _out, BezierTMesh& _mesh, BaseObject &_baseObj);

    /// Write binary mesh data to file
    bool writeBinaryData(std::ostream& _out, BezierTMesh& _mesh);

    /// Write ASCII mesh data to file
    bool writeASCIIData(std::ostream& _out, BezierTMesh& _mesh);

    //Option Widgets
    QWidget* loadOptions_;
    QWidget* saveOptions_;

    QCheckBox*   saveBinary_;
    QLabel*      savePrecisionLabel_;
    QSpinBox*    savePrecision_;
    QPushButton* saveDefaultButton_;

    QCheckBox*   loadCheckManifold_;
    QPushButton* loadDefaultButton_;

    unsigned int userReadOptions_;
    unsigned int userWriteOptions_;

    BTMImporter::ObjectOptionsE trimeshOptions_;

};

