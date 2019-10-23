#pragma once

#include <ObjectTypes/BezierTriangleMesh/BezierTriangleMesh.hh>

namespace betri
{

class Decimation
{
public:

    Decimation(BezierTMesh &mesh, BezierTMesh &ctrl)
    {
        prepare();
    }

    ~Decimation()
    {
        cleanup();
    }

    void decimate(bool stepwise=false);

    void step();

private:

    void prepare();
    void cleanup();

    void repairQ();

};

}
