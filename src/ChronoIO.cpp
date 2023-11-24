#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include "VectorUtility.h"
#include "ChronoIO.h"
#include "RFT.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChContactContainerDVI.h"

using namespace chrono;




// dump pos vel acc, rot, ome, and rot_acc for the body
std::ofstream nodinfofile("snake.mov");
void ChronoIOManager::DumpNodInfo() {
  nodinfofile << std::setprecision(8);
  nodinfofile << std::scientific;
  const size_t nnodes = mChSys->Get_bodylist()->size();
  for (unsigned int i = 0; i < nnodes; ++i) {
	  auto curbody = mChSys->Get_bodylist()->at(i);
    if (curbody->GetIdentifier() < 0)
      continue;
    nodinfofile << curbody->GetIdentifier() << " ";
    nodinfofile << curbody->GetPos() << " " << curbody->GetPos_dt() << " ";
    ChQuaternion<> rotquat = curbody->GetRot();
    // needs to conjugate to satisfy the matlab convention
    rotquat.Conjugate();
    nodinfofile << rotquat << " ";
    // now output the angular velocity
    nodinfofile << curbody->GetWvel_par() << " ";
    nodinfofile << mChSys->GetChTime() << "\n";
  }
  nodinfofile.flush();
}


// dump pos vel acc, rot, ome, and rot_acc for the link
std::ofstream jntinfofile("snake.jnt");
void ChronoIOManager::DumpJntInfo() {
  jntinfofile << std::setprecision(8);
  jntinfofile << std::scientific;
  std::vector<std::shared_ptr<ChLink> >::iterator itr = mChSys->Get_linklist()->begin();
  for (; itr != mChSys->Get_linklist()->end(); ++itr) {
    ChVector<> localforce = (*itr)->Get_react_force();
    //double localtorque = ((chrono::ChLinkEngine*)*itr)->Get_mot_torque();
	double localtorque = 0;
	if (auto linkEngineIter = std::dynamic_pointer_cast<ChLinkEngine>(*itr)) {
		localtorque = linkEngineIter->Get_mot_torque();
	}
    jntinfofile << (*itr)->GetIdentifier() << " ";
    jntinfofile << localforce << " " << localtorque << " ";
    jntinfofile << mChSys->GetChTime() << "\n";
    // jntinfofile << (*itr)->GetLinkRelativeCoords().rot.Rotate(localforce) << " " << (*itr)->GetLinkRelativeCoords().rot.Rotate(localtorque) << "\n";
  }
  jntinfofile.flush();
}

std::ofstream cotinfofile("snake.cot");
void ChronoIOManager::DumpContact() {
  std::vector<std::shared_ptr<ChBody> >::iterator ibody = mChSys->Get_bodylist()->begin(); 
  for (; ibody != mChSys->Get_bodylist()->end(); ++ibody) {
	  cotinfofile << (*ibody)->GetIdentifier() << " ";
	  cotinfofile << (*ibody)->GetContactForce() << " ";
	  cotinfofile << mChSys->GetChTime() << "\n";
  }
}

std::ofstream rftinfofile("snake.rft");
void ChronoIOManager::DumpRFTInfo() {
  const size_t nnodes = mRFTBodyList->size();
  for (int i = 0; i < nnodes; ++i) {
    rftinfofile << (*mRFTBodyList)[i].GetChBody()->GetIdentifier() << " ";
    rftinfofile << (*mRFTBodyList)[i].flist_ << "\n";
  }
}
