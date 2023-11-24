#pragma once
#include <vector>
#include <chrono/core/ChVector.h>

namespace chrono
{
    class ChBody;
}

void SetRFTMesh(chrono::ChBody *pbody, std::vector<chrono::ChVector<> > &, std::vector<chrono::ChVector<> > &, std::vector<double> &);