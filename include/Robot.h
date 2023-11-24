#ifndef ROBOT_H
#define ROBOT_H
#include <vector>
#include "chrono/core/ChMath.h" 
#include "chrono/physics/ChBody.h" 
#include "chrono/physics/ChSystem.h"
#include "SnakeSimDefines.h"
class RFTBody;
class RFTSystem;
class RobotController;

class SnakeControlSet
{
public:
    // shape params
    double k;
    double A;
    // frequency
    double w;
    double h;
};

struct GlobalControlSet {
  double gTimeStep;
  SnakeControlSet snakeParams;
};

class ChronoRobotBuilder
{
    // needs to be provided at initialization
	chrono::ChSystem *mChSys;
    // needed at the building robot
    SnakeControlSet *mSnakeParams;
    // maybe needed
    std::vector<RFTBody> mRFTBodylist;
    std::vector<chrono::ChBody*> mCollisionObjs;
    RFTSystem *mRFT;
    RobotController *mController;
public:
	ChronoRobotBuilder(chrono::ChSystem * otherSys);
    void BuildRobot(int,double,double,double,double);
    void ResetRobot();
    void SetControlSet(SnakeControlSet *snakeParams) {mSnakeParams = snakeParams;}
    void SetRFTSystems(RFTSystem *rft) { mRFT = rft; }
    RobotController *GetController() { return mController; }
    std::vector<RFTBody>& getRFTBodyList()
    {
        return mRFTBodylist;
    }
    chrono::ChVector<> GetRobotCoMPosition();
    void BuildBoard(double, double, double,double, double, double, double, double,int,int);
    void BuildBoard_onePeg(double xPos, double zPos, double pegFriction, double boxSx);
    void SetCollide(bool);

	void SetDEM_Coef(double cor, double YoungModulus, double PoissonRatio, double ly);
	double dem_cor;
	double dem_YoungModulus;
	double dem_PoissonRatio;
	double kn;
	double gn;
};


#endif
