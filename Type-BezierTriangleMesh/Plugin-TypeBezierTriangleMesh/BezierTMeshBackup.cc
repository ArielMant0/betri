#include "BezierTMeshBackup.hh"

//-----------------------------------------------------------------------------

BezierTMeshBackup::BezierTMeshBackup(BTMeshObject* _meshObj, QString _name, UpdateType _type)
	: BaseBackup(_meshObj, _name, _type), meshObj_(_meshObj)
{
//   std::cerr << "Create TriMeshBackup with name:" << name_.toStdString() << "(id : " << id_ << ")" << std::endl;
	meshBackup_ = new BezierTMesh(*(PluginFunctions::btMesh(meshObj_)));
}

//-----------------------------------------------------------------------------

BezierTMeshBackup::~BezierTMeshBackup()
{
//   std::cerr << "Delete TriMeshBackup with name:" << name_.toStdString() << "(id : " << id_ << ")" << std::endl;
	delete meshBackup_;
}

//-----------------------------------------------------------------------------

void BezierTMeshBackup::apply()
{

	//first apply the baseBackup
	BaseBackup::apply();

//   std::cerr << "Apply TriMeshBackup with name:" << name_.toStdString() << "(id : " << id_ << ")" << std::endl;
	*(PluginFunctions::btMesh(meshObj_)) = *meshBackup_;
}

//-----------------------------------------------------------------------------
