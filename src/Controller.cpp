#include "Controller.h"
#include "Robot.h"
#include "physics/ChLinkEngine.h"
#include "ChFunction_SquareWave.h"
#include "ChFunction_PID.h"
#include "ChFunction_Data.h"
#include <iostream>
#include <string>
#include <sstream>

using namespace chrono;

void ReadCSV(std::ifstream& inputfile, std::vector<double> &time, std::vector<double> &data)
{
    std::string str;
    int i = 0;
    while (std::getline(inputfile, str, '\n'))
    {
        std::istringstream myline(str);
        std::string mynum;
        int j = 0;
        std::vector<double> tmpline(256);
        while (std::getline(myline, mynum, ','))
        {
            tmpline[j] = strtod(mynum.c_str(), NULL);
            ++j;
        }
        time.push_back(tmpline[0]);
        data.push_back(tmpline[1]);
        ++i;
    }
}

RobotController::RobotController() : mSnakeParams(NULL)
{
}

void RobotController::PositionControl()
{
    double kk = mSnakeParams->k;
    double AA = mSnakeParams->A;
    double ww = mSnakeParams->w;

    for (int i = 0; i < mEngines.size(); ++i)
    {
        ChLinkEngine *mylink = mEngines[i];
        mylink->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
        auto rotfunc = std::make_shared<ChFunction_Sine>(double(i) / kk * CH_C_2PI, ww, AA);  // removing vertical lifting segments
//		auto rotfunc = std::make_shared<ChFunction_Sine>(double(2 * i) / kk * CH_C_2PI, ww, AA);  // if vertical lifting segments
        mylink->Set_rot_funct(rotfunc);
    }
}

void RobotController::ActiveLifting()
{
    for (int i = 0; i < mEngines.size(); ++i)
    {
        // get the current angle of bending
        ChLinkEngine *mylink = mEngines[i];
        ChBody *b1 = mylink->GetMarker1()->GetBody();
        ChBody *b2 = mylink->GetMarker2()->GetBody();
        double angle = fabs(mylink->Get_mot_rot());
        // std::cout << angle << std::endl;
        if (angle > 0.4)
        {
            // disable the collision on the links
            b1->SetCollide(false);
            b2->SetCollide(false);
        }
        else
        {
            b1->SetCollide(true);
            b2->SetCollide(true);

        }
        // std::cout << b1->GetIdentifier() << " " << b1->GetCollide() << std::endl;
        // std::cout << b2->GetIdentifier() << " " << b2->GetCollide() << std::endl;
        // std::cout << id1 << " " << id2 << " " << mylink->Get_mot_rot() << std::endl;
    }
}

void RobotController::TorqueControl()
{
    double kk = mSnakeParams->k;
    double AA = mSnakeParams->A;
    double ww = mSnakeParams->w;

    for (int i = 0; i < mEngines.size(); ++i)
    {
        ChLinkEngine *mylink = mEngines[i];
//        double torquelimit = 1.1;
        double torquelimit = 1.1; 
        mylink->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
        auto rotfunc = std::make_shared<ChFunction_Sine>(double(i * 2) / kk * CH_C_2PI, ww, AA);
        mylink->Set_rot_funct(rotfunc);
        mylink->GetLimit_Rz()->Set_active(true);

        mylink->GetLimit_Rz()->Set_min(-CH_C_PI_2);
        mylink->GetLimit_Rz()->Set_max( CH_C_PI_2);        

        // now open the file for input
        /*
        std::stringstream ss("");
        ss << "mot" << i + 1 << ".dat";
        std::ifstream file(ss.str());
        if (!file.is_open())
        {
            exit(0);
        }
        std::vector<double> time;
        std::vector<double> data;

        ReadCSV(file, time, data);
        file.close();
        ChSharedPtr<ChFunction_Data> torqfunct(new ChFunction_Data(time, data));
        mylink->Set_tor_funct(torqfunct);
        */
		auto torqfunct = std::make_shared<ChFunction_PID>(1.1, 0, 0, mylink);
        //ChSharedPtr<ChFunction_PID> torqfunct(new ChFunction_PID(2, 0, 0.1, mylink));
        torqfunct->mDt = 1e-3;
        torqfunct->SetMax( torquelimit);
        torqfunct->SetMin(-torquelimit);
        mylink->Set_tor_funct(torqfunct);
        /*
        double ratio = 0.5;
        double period = 1 / ww;
        double offset = double(k) / kk;
        double value =  0.3 - 0.3 * cos(CH_C_2PI * double(k) / kk);
        if (int(k / 2) % 2 == 0)
        {
        value = -value;
        offset = offset + period * ratio;
        }
        ChSharedPtr<ChFunction_SquareWave> torqfunc(new ChFunction_SquareWave(ratio, period, value, offset));
        mylink->Set_tor_funct(torqfunc);
        */
    }
}

void RobotController::UpdatePID(double curr_t)
{
	for (int i = 0; i < mEngines.size(); ++i)
	{
		ChLinkEngine *mylink = mEngines[i];
		if (auto torcFuncPID = std::dynamic_pointer_cast<ChFunction_PID>(mylink->Get_tor_funct())) {
			torcFuncPID->Update(curr_t);
		}
	}
}
