#pragma once
#include "VectorUtility.h"
#include "mesher.h"

namespace chrono
{
    class ChBody;
}

namespace irr
{
    class ChIrrApp;
}

class RFTBody
{
public:
    RFTBody(chrono::ChBody* p) : chbody_(p) { Resize(30); }
    // force on each of the piece
    std::vector<chrono::ChVector<> > flist_;
    // the position of each piece in CoG frame
    std::vector<chrono::ChVector<> > plist_;
    // the orientation of each piece in CoG frame
    std::vector<chrono::ChVector<> > olist_;
    // area of each of the piece;
    std::vector<double> alist_;
    void SetMesh()
    {
        SetRFTMesh(chbody_, plist_, olist_, alist_);
    }
    void Resize(int n) { 
        npiece_ = n;
        flist_.resize(n);
        plist_.resize(n);
        olist_.resize(n);
        alist_.resize(n);
        SetMesh();
    }
    int GetNumPieces() { return npiece_; }
    chrono::ChBody* GetChBody() { return chbody_; }
private:
    size_t npiece_;
    chrono::ChBody* chbody_;
};

class RFTSystem
{
    chrono::ChVector<> ydir_;
    chrono::ChVector<> xdir_;
    chrono::ChVector<> zdir_;
    irr::ChIrrApp *mApp;
    double ffac_;
    void InteractPiece(const chrono::ChVector<>& pos, const chrono::ChVector<>& vel, const chrono::ChVector<>& ori, const chrono::ChVector<>& nor, const chrono::ChVector<> &fow, double area, double pdist, chrono::ChVector<>& force);
public:
    RFTSystem(irr::ChIrrApp *app);
    ~RFTSystem();
    void InteractExt(RFTBody& body);
    void AddHeadDrag(RFTBody& body);
};
