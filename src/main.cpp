// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

#include "RFT.h"
#include "GuiControl.h"
#include "Robot.h"
#include "ChronoIO.h"
#include "Controller.h"
#include "chrono/core/ChFileutils.h"
#include "SnakeSimDefines.h" // include this last

#ifdef USE_IRR
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrAppInterface.h"
#include "chrono_irrlicht/ChIrrCamera.h"
#endif

using namespace chrono;
using namespace irrlicht;

//--------------------------------------------------------------------------------
//----  input parameters -------------------------------------------------------
//--------------------------------------------------------------------------------

double sp = 0.14;
double wallPosx = 1;
double stopTime = 10;

double saveVideo = 0;
double RFTstartTime = 1.7;
double pegFriction = 0;

int kNseg = 25;
double lx = 0.2;
double ly = 0.1;

double lz = 0.1;
double boxSx = 0.1;
int kNpeg = 51;
double zOffset = 1;

// pne peg simulation
double xOnePeg = 0;
double zOnePeg = 0;


//--------------------------------------------------------------------------------
//----  setting parameters -------------------------------------------------------
//--------------------------------------------------------------------------------
double del_time0 = .05; // far from contact
double del_time = .01;	// during contact, may be assigned at command line
int n_iter = 50;
int timeIntegrationType = 0; // 0: Anitescu, 1: Tasora
double recoverySpeed = 0.6; // default value in ChSystem is 0.6
double colEnvelop = 0.03;	// / default envelop is default_model_envelope = 0.03
double colMargin = 0.01;	// / default margin is default_safe_margin = 0.01
double sinMult = 0.1;
double cosMult = 5.0;

//--------------------------------------------------------------------------------
//----  material properties (DEM) -------------------------------------------------------
//--------------------------------------------------------------------------------
double E = 2e6;
double nu = 0.4;
double cor = 1; // no restitution
//--------------------------------------------------------------------------------
#ifdef USE_DEM
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/physics/ChContactContainerDEM.h"
#include "chrono/solver/ChSolverDEM.h"
ChSystemDEM::ContactForceModel contact_force_model =
ChSystemDEM::ContactForceModel::PlainCoulomb;
ChSystemDEM::TangentialDisplacementModel tangential_displ_mode =
ChSystemDEM::TangentialDisplacementModel::MultiStep;
#endif


class MyBroadPhaseCallback : public collision::ChBroadPhaseCallback {
public:
	/// Callback used to report 'near enough' pairs of models.
	/// This must be implemented by a child class of ChBroadPhaseCallback.
	/// Return false to skip narrow-phase contact generation for this pair of bodies.
	bool BroadCallback(collision::ChCollisionModel* mmodelA,///< pass 1st model
			collision::ChCollisionModel* mmodelB	///< pass 2nd model
			) {
		int idA = mmodelA->GetPhysicsItem()->GetIdentifier();
		int idB = mmodelB->GetPhysicsItem()->GetIdentifier();
		if ((idA < 0 && idB >= 0) || (idB < 0 && idA >= 0))
			return true;
		else
			return false;
	}
	;
};

class MyNarrowPhaseCallback : public collision::ChNarrowPhaseCallback {
	virtual void NarrowCallback(collision::ChCollisionInfo& mcontactinfo) {
#ifndef USE_DEM
		return;
#endif
		collision::ChCollisionModel* mmodelA = mcontactinfo.modelA;
		collision::ChCollisionModel* mmodelB = mcontactinfo.modelB;
		auto bbodyA = dynamic_cast<ChBody*>(mmodelA->GetPhysicsItem());
		auto bbodyB = dynamic_cast<ChBody*>(mmodelB->GetPhysicsItem());
		int midA = bbodyA->GetIdentifier();
		int midB = bbodyB->GetIdentifier();
		int idA = std::max(midA, midB);
		int idB = std::min(midA, midB);
		ChBody* bodyA;
		ChBody* bodyB;
		collision::ChCollisionModel* modelA;
		collision::ChCollisionModel* modelB;
		if (idA == midA) {
			bodyA = bbodyA;
			bodyB = bbodyB;
			modelA = mmodelA;
			modelB = mmodelB;
		}
		else {
			bodyA = bbodyB;
			bodyB = bbodyA;
			modelA = mmodelB;
			modelB = mmodelA;
		}
		if ((idA < 0 || idA > 999) || (idB > -2)) { // it is not a snake-peg contact, also rember that all snake pieces ids including the joint fillers are indexed below 1000
			return;
		}
		double envelopeA = modelA->GetEnvelope();
		double envelopeB = modelB->GetEnvelope();
		auto p1 = bodyA->GetPos();
		auto p2 = bodyB->GetPos();
		auto dist3 = p2 - p1;
		dist3.y = 0;
		auto cDist = dist3.Length();
		auto vN = dist3;
		vN.Normalize();
		double delta;
		if (idA < nSeg) { // body piece
			delta = cDist - (rPeg + 0.5 * lz);
		}
		else if (idA == nSeg) { // head piece
			delta = cDist - (rPeg + rHead);
		}
		else if (idA == nSeg + 1) { // tail piece
			delta = cDist - (rPeg + rTail);
		} else if (idA >= 100) { // gap filler
			delta = cDist - (rPeg + rJoint);	
		}
		if (delta > 0) {
			return;
		}
		mcontactinfo.distance = delta;
		mcontactinfo.vpA = mcontactinfo.vpB = p2 - vN * rPeg;
		if (idA == midA) {
			mcontactinfo.vN = vN;
		}
		else {
			mcontactinfo.vN = -vN;
		}
	}	
public:
	void SetSnakeParameters(int ohter_nSeg, double other_rPeg, double other_lz) {
		nSeg = ohter_nSeg;
		rPeg = other_rPeg;
		lz = other_lz;
		rTail = 0.5 * lz;
		rHead = 0.5 * lz;
		rJoint = 0.5 * lz;
	}
private:
	int nSeg;
	double rPeg;
	double rTail;
	double rHead;
	double rJoint;
	double lz;
};

void ApplyRFTForce(std::vector<RFTBody>& bodylist, RFTSystem& rsystem) {
	const size_t nnodes = bodylist.size();
	for (unsigned int i = 0; i < nnodes; ++i) {
		bodylist[i].GetChBody()->Empty_forces_accumulators();
		rsystem.InteractExt(bodylist[i]);
	}
}

void InitializeSystemFromInput(int argc, char *argv[], CHSYSTEM & my_system, SnakeControlSet *params) {
	if (argc > 4) {
		const char* text = argv[4];
		sp = atof(text);
	}
	if (argc > 5) {
		const char* text = argv[5];
		wallPosx = atof(text);
	}
	if (argc > 6) {
		const char* text = argv[6];
		stopTime = atof(text);
	}

	if (argc > 7) {
		const char* text = argv[7];
		saveVideo = atof(text);
	}
	if (argc > 8) {
		const char* text = argv[8];
		RFTstartTime = atof(text);
	}
	if (argc > 9) {
		const char* text = argv[9];
		pegFriction = atof(text);
	}

	if (argc > 10) {
		const char* text = argv[10];
		kNseg = atof(text);
	}
	if (argc > 11) {
		const char* text = argv[11];
		lx = atof(text);
	}
	if (argc > 12) {
		const char* text = argv[12];
		ly = atof(text);
	}

	if (argc > 13) {
		const char* text = argv[13];
		lz = atof(text);
	}
	if (argc > 14) {
		const char* text = argv[14];
		boxSx = atof(text);
	}
	if (argc > 15) {
		const char* text = argv[15];
		kNpeg = atof(text);
	}

	if (argc > 16) {
		const char* text = argv[16];
		zOffset = atof(text);
	}

	//
	if (argc > 17) {
		const char* text = argv[17];
		del_time = atof(text);
	}
	if (argc > 18) {
		const char* text = argv[18];
		n_iter = atoi(text);
	}
	if (argc > 19) {
		const char* text = argv[19];
		timeIntegrationType = atoi(text);
	}
	if (argc > 20) {
		const char* text = argv[20];
		recoverySpeed = atof(text);
	}
	if (argc > 21) {
		const char* text = argv[21];
		colEnvelop = atof(text);
	}
	if (argc > 22) {
		const char* text = argv[22];
		colMargin = atof(text);
	}
	if (argc > 23) {
		const char* text = argv[23];
		sinMult = atof(text);
	}
	if (argc > 24) {
		const char* text = argv[24];
		cosMult = atof(text);
	}
	if (argc > 25) {
		const char* text = argv[25];
		xOnePeg = atof(text);
	}
	if (argc > 26) {
		const char* text = argv[26];
		zOnePeg = atof(text);
	}

	// **********************************
	// Create a ChronoENGINE physical system

	switch (timeIntegrationType) {
	case 0:
		my_system.SetIntegrationType(ChSystem::INT_ANITESCU);
		break;
	case 1:
		my_system.SetIntegrationType(ChSystem::INT_TASORA);
		break;
	case 2:
		my_system.SetIntegrationType(ChSystem::INT_HHT);
		break;
	case 3:
		my_system.SetIntegrationType(ChSystem::INT_NEWMARK);
		break;
	case 4:
		my_system.SetIntegrationType(ChSystem::INT_RUNGEKUTTA45);
		break;
	case 5:
		my_system.SetIntegrationType(ChSystem::INT_TRAPEZOIDAL);
		break;
	case 6:
		my_system.SetIntegrationType(ChSystem::INT_EULER_IMPLICIT);
		break;
	default:
		break;
	}
}

void InitializeSnakeFromInput(int argc, char *argv[], SnakeControlSet *params) {
	if (argc > 1) {
		const char* text = argv[1];
		params->k = atoi(text);
	}
	if (argc > 2) {
		const char* text = argv[2];
		params->A = atof(text);
	}
	if (argc > 3) {
		const char* text = argv[3];
		params->w = atof(text);
	}

	params->h = -.26;
}

#ifdef USE_IRR
void screenshot(int stepsPerFrame, ChIrrApp* app) {
	static int frameNum = 0;
	if (frameNum % stepsPerFrame == 0) {
		ChFileutils::MakeDirectory("video_capture");
		auto image = app->GetVideoDriver()->createScreenShot();
		char filename[100];
		//sprintf(filename, "video_capture/screenshot%05d.bmp", (frameNum + 1) / stepsPerFrame);
		sprintf(filename, "video_capture/screenshot%05d.jpeg",
				(frameNum + 1) / stepsPerFrame);
		if (image)
			app->GetVideoDriver()->writeImageToFile(image, filename);
		image->drop();
	}
	frameNum++;

}
#endif

void SaveToMovie(double fps) {
	char comm[100];
	sprintf(comm, "ffmpeg -framerate %d -i ", fps);
	//strcat(comm, "video_capture/screenshot%05d.jpeg -c:v libxvid -b:v 100000k video_capture/outVid.avi");
	strcat(comm,
			"video_capture/screenshot%05d.jpeg -c:v libxvid -q 0 video_capture/outVid.avi");
	//exit(-1);
	system(comm);
}

int main(int argc, char *argv[]) {
	SetChronoDataPath(CHRONO_DATA_DIR);
	GlobalControlSet simParams;
	CHSYSTEM my_system;

	InitializeSystemFromInput(argc, argv, my_system, &simParams.snakeParams);
	simParams.gTimeStep = del_time;

	my_system.SetMaxItersSolverSpeed(n_iter);
	my_system.SetMaxItersSolverStab(n_iter);
	my_system.SetSolverWarmStarting(false); // my addition
	my_system.SetSolverType(ChSystem::SOLVER_SOR);
	my_system.SetTol(1e-8);
	my_system.Set_G_acc(ChVector<>(0, -9.8 * 0, 0));
	MyBroadPhaseCallback * myBroadPhaseCallback = new MyBroadPhaseCallback;
	my_system.GetCollisionSystem()->SetBroadPhaseCallback(myBroadPhaseCallback);
	MyNarrowPhaseCallback myNarrowPhaseCallback;
	myNarrowPhaseCallback.SetSnakeParameters(kNseg, 0.5 * boxSx, lz);
	my_system.GetCollisionSystem()->SetNarrowPhaseCallback(&myNarrowPhaseCallback);
	collision::ChCollisionModel::SetDefaultSuggestedEnvelope(colEnvelop);
	collision::ChCollisionModel::SetDefaultSuggestedMargin(colMargin);
#ifdef USE_DEM
	my_system.SetContactForceModel(contact_force_model);
#endif

	// create a gui application with the chrono system
#ifdef USE_IRR
	ChIrrApp application(&my_system, L"A simple RFT example",
			irr::core::dimension2d<irr::u32>(1024, 768), false, true,
			irr::video::EDT_OPENGL);
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());

	// for one wave: use camera position (3,5,0) look at target position (4,0,0);
	// for two wave: use camera position (4,5,0) look at target position (6,0,0);

	/*ChIrrWizard::add_typical_Camera(application.GetDevice(),
			irr::core::vector3df(0.5, 1.5, 0), irr::core::vector3df(2, 0, 0));*/
	ChIrrWizard::add_typical_Camera(application.GetDevice(),
		irr::core::vector3df(1.3512, 1.0, 0.0019), irr::core::vector3df(1.3512, 0, 0.0019));

//	ChIrrWizard::add_typical_Camera(application.GetDevice(),
//			irr::core::vector3df(1, 4, 0), irr::core::vector3df(3, 0, 0));
	auto cur_cam = application.GetSceneManager()->getActiveCamera();
	cur_cam->setRotation(irr::core::vector3df(0, 90, 0));

	MyEventReceiver receiver(&application, &simParams);
	application.SetUserEventReceiver(&receiver);
	//// Create a RFT ground, set the scaling factor to be 1;
	RFTSystem rsystem(&application, sinMult, cosMult);
#else
	RFTSystem rsystem(sinMult, cosMult);
#endif

	// now let us build the robot;
	ChronoRobotBuilder theSnake(&my_system);
	theSnake.SetRFTSystems(&rsystem);
	InitializeSnakeFromInput(argc, argv, &simParams.snakeParams);

#ifdef USE_IRR
	receiver.UpdateText();
#endif

	//------------------------------------------------
	// ----------------- Set DEM parameters ----------
#ifdef USE_DEM
	my_system.UseMaterialProperties(false);
	theSnake.SetDEM_Coef(cor, E, nu, ly); 
	printf("\n** DEM **\n\n");
#else
	printf("\n** DVI **\n\n");
#endif
	//------------------------------------------------


	theSnake.SetControlSet(&(simParams.snakeParams));
	theSnake.BuildRobot(kNseg, lx, ly, lz, pegFriction);

	// multiple peg case
	// -----------------
	//theSnake.BuildBoard(sp, wallPosx, zOffset, pegFriction, lx, ly, lz, boxSx, kNseg, kNpeg);

	// single peg case
	// -----------------
	theSnake.BuildBoard_onePeg(xOnePeg, zOnePeg, pegFriction, boxSx);

	theSnake.SetCollide(false);
	del_time0 = simParams.gTimeStep;
	double sim_dt = del_time0;
#ifdef USE_IRR
	application.AssetBindAll();
	application.AssetUpdateAll();
	application.SetStepManage(true);
	application.SetTimestep(sim_dt);
	application.SetTryRealtime(false);
#endif
	// get all the RFT bodylist to interact
	std::vector<RFTBody>& bodylist = theSnake.getRFTBodyList();

	// set io
	ChronoIOManager ioManage(&my_system, &theSnake.getRFTBodyList());

	// get the controller
	RobotController *control = theSnake.GetController();
	control->PositionControl();
	//control->TorqueControl();

	// screen capture?
	int savestep = 1; //1e-2 / simParams.gTimeStep; change timestep for output here
	int irrlichtStep = 1; //1e-2 / simParams.gTimeStep; change timestep for output here
	double fps = 30;

	int count = 1;
	bool switchparams = true;

	while (true) {
		// the core simulation part
		/* if (my_system.GetChTime() >= 1.0)
		 {
		 ApplyRFTForce(bodylist, rsystem);
		 }
		 */

		/*
		 if (abs(my_system.GetChTime() - 1.0) < 0.01)
		 {
		 //control->TorqueControl();
		 }
		 */

		if ((my_system.GetChTime() > 9) && (my_system.GetChTime() < 20.0)) {
			sim_dt = simParams.gTimeStep;
		} else if (my_system.GetChTime() > 20.0) {
			sim_dt = del_time0;
		}

#ifdef USE_IRR
		if (!application.GetDevice()->run()) {
			break;
		}
		application.SetTimestep(sim_dt);
		if (saveVideo) {
			//application.SetVideoframeSave(true);
			//application.SetVideoframeSaveInterval(savestep);
			screenshot((int) (1.0 / (.01) / fps), &application);
		}
		application.DoStep();

		// -------------------------------------------------------------
		// if TorqueControl is used, you need to call the following line
		// --------------------------------------------------------------
		//control->UpdatePID(my_system.GetChTime());

		// ChVector<> cam_pos = theSnake.GetRobotCoMPosition();
		// scene::ICameraSceneNode* cur_cam = application.GetSceneManager()->getActiveCamera();
		// cur_cam->setPosition(core::vector3df(cam_pos.x, cam_pos.y + 5.2, cam_pos.z));
		// cur_cam->setTarget(core::vector3df(cam_pos.x, cam_pos.y, cam_pos.z));

		// io control
		if (count % irrlichtStep == 0) {
			application.GetVideoDriver()->beginScene(true, true,
					irr::video::SColor(255, 140, 161, 192));
			application.DrawAll();

			// draw a grid to help visualizattion
			ChIrrTools::drawGrid(application.GetVideoDriver(), 5, 5, 100, 100,
					ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
					irr::video::SColor(255, 80, 100, 100), true);
			ChIrrTools::drawAllContactPoints(my_system,
					application.GetVideoDriver(), 1,
					ChIrrTools::eCh_ContactsDrawMode::CONTACT_FORCES);
			application.GetVideoDriver()->endScene();
		}
#else
		my_system.DoStepDynamics(sim_dt);
#endif

		//// feifei
		// if (my_system.GetChTime() >= 1.72) { // OneWaveNoSlip
		//   ApplyRFTForce(bodylist, rsystem);

		// jennifer
		if (my_system.GetChTime() >= RFTstartTime) { // OneWaveSlip
			ApplyRFTForce(bodylist, rsystem);

			// control->TorqueControl();
			// }

		}

		// io control
		if (count % savestep == 0) {
			//control->ActiveLifting();
			if (count / savestep % 100 == 0) {
				std::cout << std::fixed << std::setprecision(4)
						<< my_system.GetChTime() << std::endl;
			}
			ioManage.DumpNodInfo();
			ioManage.DumpJntInfo();
			ioManage.DumpContact();
		}

		if (my_system.GetChTime() > 1.0 && switchparams) {
			theSnake.SetCollide(true);
			switchparams = false;
		}
#ifdef USE_IRR
		if (!application.GetPaused())
			++count;
#else
		++count;
#endif

		if (my_system.GetChTime() >= stopTime) {
			break;
		}
	}
	if (saveVideo) {
		SaveToMovie(fps);
	}
	return 0;
}

