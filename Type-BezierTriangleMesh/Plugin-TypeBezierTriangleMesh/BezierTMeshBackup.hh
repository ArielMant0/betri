#pragma once

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>
#include <OpenFlipper/common/BaseBackup.hh>

/**
 * @brief Class that encapsulates a backup
 */
class BezierTMeshBackup : public BaseBackup
{

public:
	BezierTMeshBackup(BTMeshObject* _meshObj, QString _name, UpdateType _type);
    ~BezierTMeshBackup();

public:
    void apply();

private:
	BTMeshObject* meshObj_;
    BezierTMesh* meshBackup_;
};
