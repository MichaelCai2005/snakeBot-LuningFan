#include "Robot.h"
#include "RFT.h"
#include "assets/ChColorAsset.h"
#include "chrono/physics/ChBodyEasy.h" 
#include "ChFunction_SquareWave.h"
#include "Controller.h"
using namespace chrono;

ChronoRobotBuilder::ChronoRobotBuilder(ChSystem * otherSys) :
		mChSys(otherSys), mSnakeParams(NULL), mRFT(NULL) {
	dem_cor = 0.1;
	dem_YoungModulus = 1e6;
}

double SegA(double s, double A) {
	// relative frame rotation between each seg;
	return A * cos(CH_C_2PI * s);
}
/*
 void
 ChronoRobotBuilder::BuildRobot()
 {
 if (true)

 {
 // the relation between curvature and the
 // thm * 2pi = AA * kk / delta_k
 if (!mSnakeParams)
 {
 std::cout << "no control params specified!\n" << std::endl;
 return;
 }

 double AA = mSnakeParams->A;
 double kk = mSnakeParams->k;
 double ww = mSnakeParams->w;
 double hh = mSnakeParams->h;

 std::cout << kk << "\t" << AA << "\t" << ww << std::endl;
 const int kNseg = 12 + 11;
 mController = new RobotController();
 mController->SetControlSet(mSnakeParams);


 const double friction = 0.2;
 std::vector<ChSharedBodyPtr> body_container_;
 ChSharedPtr<ChBodyEasyBox> ground(new ChBodyEasyBox(20, 1.0, 20, 1e3, false, false));
 ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset);
 mcolor->SetColor(ChColor(0.1f, 0.1f, 0.8f));
 ground->SetPos(ChVector<>(0, -0.5, 0.));
 ground->AddAsset(mcolor);
 ground->SetBodyFixed(true);
 ground->SetIdentifier(-1);
 ground->SetFriction(friction);
 mChSys->AddBody(ground);


 const double lx = 0.10;
 const double ly = 0.05;
 const double lz = 0.05;
 const double rho = 1450;
 const double ypos = hh;
 const double zpos = 0.25;
 double lastx = 0;

 // construct the body
 // the initial frame aligned with world frame
 ChFrame<> lastframe(ChVector<>(lastx, ypos, zpos));
 for (int k = 0; k < kNseg; ++k)
 {
 // translate and rotate
 if (k % 2 == 0)
 {
 ChSharedPtr<ChBodyEasyBox> chShape(new ChBodyEasyBox(lx, ly, lz, rho, true, true));
 body_container_.push_back(chShape);
 chShape->SetIdentifier(k);
 chShape->SetFriction(friction);
 mChSys->AddBody(chShape);
 mRFTBodylist.push_back(RFTBody(chShape.get()));

 chShape->SetPos(lastframe * ChVector<>(0.5 * lx, 0, 0));
 chShape->SetRot(lastframe.GetRot());
 ChVector<> enginePos(lastframe * ChVector<>(0, 0, 0));

 if (k > 1)
 {
 // add the engine
 ChSharedPtr<ChLinkEngine> mylink(new ChLinkEngine);
 //mylink->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_PRISM);
 mylink->SetIdentifier(k + 1000);
 ChQuaternion<> engineOri(Q_from_AngX(CH_C_PI_2));
 mylink->Initialize(body_container_[k], body_container_[k - 2], ChCoordsys<>(enginePos, engineOri));
 mChSys->AddLink(mylink);
 mController->AddEngine(mylink.get());
 }

 ChSharedPtr<ChLinkLockPlanePlane> inplanelink(new ChLinkLockPlanePlane);
 inplanelink->SetIdentifier(10000 + k);
 inplanelink->Initialize(ground, body_container_[k], ChCoordsys<>(ChVector<>(), Q_from_AngX(CH_C_PI_2)));
 mChSys->AddLink(inplanelink);

 lastframe = ChFrame<>(ChVector<>(lx, 0, 0), Q_from_AngY(0 * SegA(double(k) / kNseg, AA))) >> lastframe;
 }

 if (k % 2 == 1 && k < kNseg - 1)
 {

 // add a cylinder segment to protect the joint
 ChSharedPtr<ChBodyEasySphere> chShape(new ChBodyEasySphere(ly / 2, rho * 0.01, true, true));
 body_container_.push_back(chShape);
 chShape->SetIdentifier(k);
 chShape->SetFriction(friction);
 mChSys->AddBody(chShape);
 ChVector<> enginePos(lastframe * ChVector<>(0, 0, 0));
 chShape->SetPos(enginePos);
 chShape->SetRot(Q_from_AngY(CH_C_PI_2));

 ChSharedPtr<ChLinkLockLock> mylink(new ChLinkLockLock);
 mylink->SetIdentifier(k + 1000);
 mylink->Initialize(body_container_[k], body_container_[k - 1], ChCoordsys<>(enginePos));
 mChSys->AddLink(mylink);

 }
 }

 if (true)
 {
 // now add the head
 ChSharedPtr<ChBodyEasyCylinder> chShape(new ChBodyEasyCylinder(0.5 * lz, ly, rho * 0.01, true, true));
 body_container_.push_back(chShape);
 chShape->SetIdentifier(kNseg);
 chShape->SetFriction(friction);
 mChSys->AddBody(chShape);
 mRFTBodylist.push_back(RFTBody(chShape.get()));
 ChVector<> enginePos(lastframe * ChVector<>(0, 0, 0));
 chShape->SetPos(enginePos);
 chShape->SetRot(Q_from_AngY(CH_C_PI_2));

 ChSharedPtr<ChLinkLockLock> mylink(new ChLinkLockLock);
 mylink->SetIdentifier(kNseg + 1000);
 mylink->Initialize(body_container_[kNseg], body_container_[kNseg - 1], ChCoordsys<>(enginePos));
 mChSys->AddLink(mylink);

 }

 }
 mController->PositionControl();
 }
 */

void ChronoRobotBuilder::BuildRobot(int kNseg, double lx, double ly, double lz,
		double pegFriction) {

	if (true) {
		// the relation between curvature and the
		// thm * 2pi = AA * kk / delta_k
		if (!mSnakeParams) {
			std::cout << "no control params specified!\n" << std::endl;
			return;
		}

		double AA = mSnakeParams->A;
		double kk = mSnakeParams->k;
		double ww = mSnakeParams->w;
		double hh = mSnakeParams->h;

		std::cout << kk << "\t" << AA << "\t" << ww << std::endl;
		// feifei
		//const int kNseg = 38 + 37; //three wave
		// const int kNseg = 25 + 24; //two wave
		//const int kNseg = 13 + 12; //one wave

		//jennifer
		//const int kNseg = 9 + 8; //one wave

		mController = new RobotController();
		mController->SetControlSet(mSnakeParams);

		//feifei
		//const double friction = 0.1;

		//jennifer -- until 2/22/16, this was 0.1
		const double friction = pegFriction; // changing this does not seem to make any difference
		int numFixLinks = 0;

#ifdef USE_DEM
		auto mat_c = std::make_shared<ChMaterialSurfaceDEM>();
		mat_c->SetRestitution(dem_cor);
		mat_c->SetYoungModulus(dem_YoungModulus);
		mat_c->SetPoissonRatio(dem_PoissonRatio);
		mat_c->SetFriction(friction);
		mat_c->SetKn(kn);
		mat_c->SetGn(gn);
#else
		auto mat_c = std::make_shared<ChMaterialSurface>();
		mat_c->SetFriction(friction);
#endif

		std::vector<std::shared_ptr<ChBody>> body_container_;
#ifdef USE_DEM
		auto ground = std::make_shared<ChBodyEasyBox>(200, 1.0, 200, 1e3, false, false, ChMaterialSurfaceBase::DEM);
#else
		auto ground =
				std::make_shared < ChBodyEasyBox
						> (200, 1.0, 200, 1e3, false, false, ChMaterialSurfaceBase::DVI);
#endif
		auto mcolor = std::make_shared<ChColorAsset>();
		mcolor->SetColor(ChColor(0.1f, 0.1f, 0.8f));
		ground->SetPos(ChVector<>(0, -0.5, 0.));
		ground->AddAsset(mcolor);
		ground->SetBodyFixed(true);
		ground->SetIdentifier(-1);
		ground->SetMaterialSurface(mat_c);

		mChSys->AddBody(ground);

		//// feifei
		//const double sf = 5;
		//const double lx = 0.04 * sf;
		//const double ly = 0.02 * sf;
		//const double lz = 0.02 * sf;
		//const double rho = 1450 / (sf * sf);
		//const double ypos = hh;
		//double lastx = 0.5;

		////jennifer
		//const double sf = 5;
		//const double lx = 0.05 * sf;
		//const double ly = 0.04 * sf;
		//const double lz = 0.04 * sf;
		//const double rho = 1450 / (sf * sf);
		//const double ypos = hh;
		//double lastx = 0.5;

		//jennifer
		const double sf = 5;
		//const double lx = 0.05 * sf;
		//const double ly = 0.045 * sf;
		//const double lz = 0.045 * sf;
		const double rho = 1200;        // / (sf * sf);
		const double ypos = hh;
		double lastx = 0.5;

		// construct the body
		// the initial frame aligned with world frame and rotated by a small angle
		double angle = 0;
		//double angle = 0.1722;
		//ChFrame<> lastframe(ChVector<>(lastx, ypos, 0.36), Q_from_AngY(angle));
		ChFrame<> initFrame(ChVector<>(lastx, ypos, 0), Q_from_AngY(angle));
		ChFrame<> lastframe(initFrame);
		//for (int k = 0; k < kNseg; ++k)
		for (int k = 0; k < kNseg; ++k) // jennifer trying make head segment separately
				{

			///////////////////////////////////////////////////////////////////////////////////
			//// for no vertical lifting segments
			double lenx = lx;
			if (k == 0) {
				lenx = .08 - 0.5 * lz;
			}
			if (k == kNseg - 1) {
				lenx = .06 - 0.5 * lz;
			}

#ifdef USE_DEM
			auto chShape = std::make_shared<ChBodyEasyBox>(lenx, ly, lz, rho, true, true, ChMaterialSurfaceBase::DEM);
#else
			auto chShape = std::make_shared<ChBodyEasyBox>(lenx, ly, lz, rho, true, true,  ChMaterialSurfaceBase::DVI);
#endif
			body_container_.push_back(chShape);
			chShape->SetIdentifier(k);
			chShape->SetMaterialSurface(mat_c);
			chShape->GetCollisionModel()->SetFamily(1);
			chShape->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
			mChSys->AddBody(chShape);
			mRFTBodylist.push_back(RFTBody(chShape.get()));

			chShape->SetPos(lastframe * ChVector<>(0.5 * (lenx), 0, 0));
			chShape->SetRot(lastframe.GetRot());
			auto mcolor = std::make_shared<ChColorAsset>();
			if (k == 0)
				mcolor->SetColor(ChColor(0.0f, 0.0f, 0.0f));
			else if (k % 2 == 0)
				mcolor->SetColor(ChColor(1.0f, 0.8f, 0.0f));
			else
				mcolor->SetColor(ChColor(0.8f, 0.8f, 0.8f));
			chShape->AddAsset(mcolor);

			ChVector<> enginePos(lastframe * ChVector<>(0, 0, 0));

			/// for experimental matching
			if (k >= 1) {
				// add the engine
				auto mylink = std::make_shared<ChLinkEngine>();
				//mylink->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_PRISM);
				mylink->SetIdentifier(k + 1000);
				ChQuaternion<> engineOri(Q_from_AngX(CH_C_PI_2));
				mylink->Initialize(body_container_[k], body_container_[k - 1],
						ChCoordsys<>(enginePos, engineOri));
				mChSys->AddLink(mylink);
				mController->AddEngine(mylink.get());
			}

			auto inplanelink = std::make_shared<ChLinkLockPlanePlane>();
			inplanelink->SetIdentifier(k + 10000);
			inplanelink->Initialize(ground, body_container_[k],
					ChCoordsys<>(ChVector<>(), Q_from_AngX(CH_C_PI_2)));
			mChSys->AddLink(inplanelink);

			lastframe = lastframe >> ChFrame<>(ChVector<>(lenx, 0, 0));
			////////////////////////////////////////////////////////////////////////////////////////

//		  /////////////////////////////////////////////////////////////////////////////////////////
//              //this is for snake with vertical lifting segments
//              // translate and rotate
//            if (k % 2 == 0)
//            {
//    			double lenx = lx;
//    			if (k == 0) {
//    				lenx = .08 - 0.5 * lz;
//    			}
//    			if (k == kNseg - 1) {
//    				lenx = .06 - 0.5 * lz;
//    			}
//#ifdef USE_DEM
//            	auto chShape = std::make_shared<ChBodyEasyBox>(lenx, ly, lz, rho, true, true, ChMaterialSurfaceBase::DEM);
//#else
//        auto chShape = std::make_shared<ChBodyEasyBox>(lenx, ly, lz, rho, true, true,  ChMaterialSurfaceBase::DVI);
//#endif
//                body_container_.push_back(chShape);
//				//chShape->SetCollide(true); // added 5/17/2016
//                chShape->SetIdentifier(k);
//                chShape->SetMaterialSurface(mat_c);
//                mChSys->AddBody(chShape);
//                mRFTBodylist.push_back(RFTBody(chShape.get()));
//
//                chShape->SetPos(lastframe * ChVector<>(0.5 * lenx, 0, 0));
//                chShape->SetRot(lastframe.GetRot());
//    			auto mcolor = std::make_shared<ChColorAsset>();
//    			if (k == 0)
//    				mcolor->SetColor(ChColor(0.0f, 0.0f, 0.0f));
//    			else if (k % 4 == 0)
//    				mcolor->SetColor(ChColor(1.0f, 0.8f, 0.0f));
//    			else if (k % 4 == 2)
//    				mcolor->SetColor(ChColor(0.8f, 0.8f, 0.8f));
//    			chShape->AddAsset(mcolor);
//
//                ChVector<> enginePos(lastframe * ChVector<>(0, 0, 0));
//
//                if (k > 1)
//                {
//                    // add the engine
//                    auto mylink = std::make_shared<ChLinkEngine>();
//                    //mylink->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_PRISM);
//                    mylink->SetIdentifier(k + 1000);
//                    ChQuaternion<> engineOri(Q_from_AngX(CH_C_PI_2));
//                    mylink->Initialize(body_container_[k], body_container_[k - 2], ChCoordsys<>(enginePos, engineOri));
//                    mChSys->AddLink(mylink);
//                    mController->AddEngine(mylink.get());
//                }
//
//				auto inplanelink = std::make_shared<ChLinkLockPlanePlane>();
//                inplanelink->SetIdentifier(k + 10000);
//                inplanelink->Initialize(ground, body_container_[k], ChCoordsys<>(ChVector<>(), Q_from_AngX(CH_C_PI_2)));
//                mChSys->AddLink(inplanelink);
//
//                lastframe = ChFrame<>(ChVector<>(lenx, 0, 0), Q_from_AngY(0 * SegA(double(k) / kNseg, AA))) >> lastframe;
//            }
//
//            if (k % 2 == 1 && k < kNseg - 1)
//            {
//
//                // add a cylinder segment to protect the joint
//#ifdef USE_DEM
//            	auto chShape = std::make_shared<ChBodyEasySphere>(ly / 2, rho * 0.01, true, true, ChMaterialSurfaceBase::DEM);
//#else
//            	auto chShape = std::make_shared<ChBodyEasySphere>(ly / 2, rho * 0.01, true, true, ChMaterialSurfaceBase::DVI);
//#endif
//                body_container_.push_back(chShape);
//                chShape->SetCollide(true);
//                chShape->SetIdentifier(k);
//                chShape->SetMaterialSurface(mat_c);
//                mChSys->AddBody(chShape);
//                ChVector<> enginePos(lastframe * ChVector<>(0, 0, 0));
//                chShape->SetPos(enginePos);
//                chShape->SetRot(Q_from_AngY(CH_C_PI_2));
//
//				auto mylink = std::make_shared<ChLinkLockLock>();
//                mylink->SetIdentifier((++numFixLinks) + 1000);
//                mylink->Initialize(body_container_[k], body_container_[k - 1], ChCoordsys<>(enginePos));
//                mChSys->AddLink(mylink);
//
//            }
//              ///////////////////////////////////////////////////////////////////////////////////////////////////////////
		}

		// -----------------------------
		// ------- head and tail -------
		// -----------------------------

		if (true) {
			// -----------------
			// ------ head -----
			// -----------------

#ifdef USE_DEM
			auto headPiece = std::make_shared<ChBodyEasySphere>(0.5 * lz, rho * 0.5, true, true, ChMaterialSurfaceBase::DEM);
#else
			auto headPiece =
					std::make_shared < ChBodyEasySphere
							> (0.5 * lz, rho * 0.5, true, true, ChMaterialSurfaceBase::DVI);
#endif
			body_container_.push_back(headPiece);
			//headPiece->SetCollide(true); // added 5/17/2016
			headPiece->SetIdentifier(kNseg);
			headPiece->SetMaterialSurface(mat_c);
			headPiece->GetCollisionModel()->SetFamily(1);
			headPiece->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
			mChSys->AddBody(headPiece);
			// mRFTBodylist.push_back(RFTBody(headPiece.get()));
			ChVector<> headPos(lastframe * ChVector<>(0, 0, 0));
			headPiece->SetPos(headPos);
			auto headFixLink = std::make_shared<ChLinkLockLock>();
			headFixLink->SetIdentifier((++numFixLinks) + 1000);
			headFixLink->Initialize(body_container_[kNseg],
					body_container_[kNseg - 1], ChCoordsys<>(headPos));
			mChSys->AddLink(headFixLink);

			auto mcolorHead = std::make_shared<ChColorAsset>();
			mcolorHead->SetColor(ChColor(1.0f, 0.8f, 0.0f));
			headPiece->AddAsset(mcolorHead);
			// -----------------
			// ------ tail -----
			// -----------------

#ifdef USE_DEM
			auto tailPiece = std::make_shared<ChBodyEasyCylinder>(0.5 * lz, ly, rho * 0.5, true, true, ChMaterialSurfaceBase::DEM);
#else
			auto tailPiece =
					std::make_shared < ChBodyEasyCylinder
							> (0.5 * lz, ly, rho * 0.5, true, true, ChMaterialSurfaceBase::DVI);
#endif
			body_container_.push_back(tailPiece);
			//tailPiece->SetCollide(true); // added 5/17/2016
			tailPiece->SetIdentifier(kNseg + 1);
			tailPiece->SetMaterialSurface(mat_c);
			tailPiece->GetCollisionModel()->SetFamily(1);
			tailPiece->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
			mChSys->AddBody(tailPiece);
			// mRFTBodylist.push_back(RFTBody(tailPiece.get()));
			ChVector<> tailPos(initFrame * ChVector<>(0, 0, 0));
			tailPiece->SetPos(tailPos);
			auto tailFixLink = std::make_shared<ChLinkLockLock>();
			tailFixLink->SetIdentifier((++numFixLinks) + 1000);
			tailFixLink->Initialize(body_container_[kNseg + 1],
					body_container_[0], ChCoordsys<>(tailPos));
			mChSys->AddLink(tailFixLink);

			auto mcolorTail = std::make_shared<ChColorAsset>();
			mcolorTail->SetColor(ChColor(0.0f, 0.0f, 0.0f));
			tailPiece->AddAsset(mcolorTail);

			// -----------------------
			// ------ gap filler -----
			// -----------------------
			for (int k = 1; k < kNseg; ++k) {
				double lenx = lx;
				if (k == 0) {
					lenx = .08 - 0.5 * lz;
				}
				if (k == kNseg - 1) {
					lenx = .06 - 0.5 * lz;
				}

				auto baseBody = body_container_[k];
				ChFrame<> baseFrame(baseBody->GetPos(), baseBody->GetRot());
#ifdef USE_DEM
				auto gapSphere = std::make_shared<ChBodyEasySphere>(0.5 * lz, .1 * rho, true, true, ChMaterialSurfaceBase::DEM);
#else
				auto gapSphere = std::make_shared<ChBodyEasySphere>(0.5 * lz, .1 * rho, true, true, ChMaterialSurfaceBase::DVI);
#endif
				body_container_.push_back(gapSphere);
				gapSphere->SetIdentifier(k + 100);
				gapSphere->SetMaterialSurface(mat_c);
				gapSphere->GetCollisionModel()->SetFamily(1);
				gapSphere->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
				mChSys->AddBody(gapSphere);
				ChVector<> gapSphere_pos(baseFrame * ChVector<>(-0.5 * lenx, 0, 0));
				gapSphere->SetPos(gapSphere_pos);
				gapSphere->SetRot(baseFrame.GetRot());
				
				auto fixLink = std::make_shared<ChLinkLockLock>();
				fixLink->SetIdentifier((++numFixLinks) + 1000);
				fixLink->Initialize(gapSphere,
					body_container_[k], ChCoordsys<>(gapSphere_pos));
				mChSys->AddLink(fixLink);
			}
		}

	}
	// You might want to change to TorqueControl
	mController->PositionControl();
}

ChVector<> ChronoRobotBuilder::GetRobotCoMPosition() {
	ChVector<> pos;
	const size_t nnode = mRFTBodylist.size();
	double total_mass = 0;
	for (int i = 0; i < nnode; ++i) {
		ChVector<> tmppos = mRFTBodylist[i].GetChBody()->GetPos();
		double tmpmass = mRFTBodylist[i].GetChBody()->GetMass();
		pos += tmppos * tmpmass;
		total_mass += tmpmass;
	}
	return pos / total_mass;
}

void ChronoRobotBuilder::BuildBoard(double spacing, double wallPosx,
		double zOffset, double pegFriction, double lx, double ly, double lz,
		double boxSx, int kNseg, int kNpeg) {
	//const int kNpeg = 2;
	double pegSeparation = spacing;

#ifdef USE_DEM
	auto mat_c = std::make_shared<ChMaterialSurfaceDEM>();
	mat_c->SetRestitution(dem_cor);
	mat_c->SetYoungModulus(dem_YoungModulus);
	mat_c->SetPoissonRatio(dem_PoissonRatio);
	mat_c->SetFriction(pegFriction);
	mat_c->SetKn(kn);
	mat_c->SetGn(gn);
#else
	auto mat_c = std::make_shared<ChMaterialSurface>();
	mat_c->SetFriction(pegFriction);
#endif

	// feifei
	/* const double sf = 5;
	 const double lx = 0.04 * sf;
	 const double ly = 0.02 * sf;
	 const double lz = 0.02 * sf;
	 double boxSx =0.1;
	 double boxSy = 1;
	 double boxSz = 5;*/

	////jennifer
	// const double sf = 5;
	// const double lx = 0.05 * sf;
	// const double ly = 0.04 * sf;
	// const double lz = 0.04 * sf;
	// double boxSx = 0.225;
	double boxSy = .2;
	double boxSz = 10;

	//jennifer
	/* const double sf = 5;
	 const double lx = 0.05 * sf;
	 const double ly = 0.045 * sf;
	 const double lz = 0.045 * sf;
	 double boxSx = 0.225;
	 double boxSy = 1;
	 double boxSz = 5;*/

	ChVector<> robotCom = GetRobotCoMPosition();
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	// feifei double wall setup -- these walls have corners!

	// 1st wall

	//double zPos = robotCom.z - pegSeparation / 2 - boxSz / 2 - boxSx / 2;
	//double xPos = (robotCom.x + wallPosx) + boxSx / 2;
	//ChSharedPtr<ChBodyEasyBox> chShape1(new ChBodyEasyBox(boxSx, boxSy, boxSz, true, true));
	//chShape1->SetCollide(true);
	//chShape1->SetIdentifier(-1);
	//chShape1->SetFriction(0.0);
	//chShape1->SetPos(ChVector<>(xPos, 0, zPos));
	//chShape1->SetBodyFixed(true);
	//mChSys->AddBody(chShape1);
	//mCollisionObjs.push_back(chShape1.get());

	// 2nd wall

	//zPos = robotCom.z + pegSeparation / 2 + boxSz / 2 + boxSx / 2;
	//xPos = (robotCom.x + wallPosx) + boxSx / 2;
	//ChSharedPtr<ChBodyEasyBox> chShape2(new ChBodyEasyBox(boxSx, boxSy, boxSz, true, true));
	//         chShape2->SetCollide(true);
	//         chShape2->SetIdentifier(-1);
	//         chShape2->SetFriction(0.0);
	//         chShape2->SetPos(ChVector<>(xPos, 0, zPos));
	//         chShape2->SetBodyFixed(true);
	//         mChSys->AddBody(chShape2);
	//         mCollisionObjs.push_back(chShape2.get());

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////// jennifer triple wall setup with rounded corners
	//if (kNpeg > 0)
	//{
	//	ChSharedPtr<ChBody> chShape1(new ChBody());
	//	chShape1->GetCollisionModel()->ClearModel();
	//	double midBoxSz = 0 * boxSx; // width of central box -- total width will be this plus boxSx

	////////// 1st wall
	//double xPos = (lx * kNseg + wallPosx) + boxSx / 2;
	////////	//double zPos = robotCom.z + zOffset - (boxSz / 2 + boxSx);  // use this for single wall (-() = on the left, +() = on the right)
	////double zPos = robotCom.z + zOffset - (pegSeparation / 2 + boxSz / 2 + boxSx/2);  // use this for double wall
	//double zPos = robotCom.z + zOffset - (midBoxSz / 2 + boxSx / 2 + pegSeparation + boxSx / 2 + boxSz / 2); // use this for triple wall
	//chShape1->GetCollisionModel()->ClearModel();

	////	 //////box 1 code ///////////////////
	//	chShape1->GetCollisionModel()->AddBox(boxSx / 2, 1, boxSz / 2, ChVector<>(0, 0, 0));  // half-lengths of box sides
	//	ChSharedPtr<ChBoxShape> boxShape1(new ChBoxShape());
	//	boxShape1->GetBoxGeometry().SetLengths(ChVector<>(boxSx, 2, boxSz)); // full length of box sides
	//	chShape1->AddAsset(boxShape1);

	////	//// add cylinders

	//	// make sure when you add collision models they are at the same position of the visualization model
	//	// previously your collision model was not at the correct position. Even though you can visualize a cylinder, the actual collision model
	//	// was not placed there.
	//	chShape1->GetCollisionModel()->AddCylinder(boxSx / 2, boxSx / 2, 2, ChVector<>(0, 0, boxSz / 2));  // radius x, radiusz, height on y
	//
	//	// this is for visualization only
	//	ChSharedPtr<ChCylinderShape> cylShape1(new ChCylinderShape());
	//	cylShape1->GetCylinderGeometry().p1 = ChVector<>(0, 1, boxSz / 2); //vector is position of top cap
	//	cylShape1->GetCylinderGeometry().p2 = ChVector<>(0, -1, boxSz / 2);   //vector is position of bottom cap
	//	cylShape1->GetCylinderGeometry().rad = boxSx / 2;
	//	chShape1->AddAsset(cylShape1);
	//
	//	chShape1->GetCollisionModel()->AddCylinder(boxSx / 2, boxSx / 2, 2, ChVector<>(0, 0, -boxSz / 2));  // radius x, radiusz, height on y
	//	ChSharedPtr<ChCylinderShape> cylShape2(new ChCylinderShape());
	//	cylShape2->GetCylinderGeometry().p1 = ChVector<>(0, 1, -boxSz / 2);    //vector is position of top cap
	//	cylShape2->GetCylinderGeometry().p2 = ChVector<>(0, -1, -boxSz / 2);   //vector is position of bottom cap
	//	cylShape2->GetCylinderGeometry().rad = boxSx / 2;
	//	chShape1->AddAsset(cylShape2);

	//	chShape1->SetBodyFixed(true);
	//	chShape1->SetIdentifier(-2);
	//	chShape1->GetMaterialSurface()->SetFriction(pegFriction);
	//	chShape1->SetPos(ChVector<>(xPos, 0, zPos));
	//	chShape1->SetCollide(true);
	//	chShape1->GetCollisionModel()->BuildModel();
	//	mChSys->AddBody(chShape1);
	//	mCollisionObjs.push_back(chShape1.get());

	//////	//// for double wall, add a second wall
	//////	////// 2nd wall
	//	ChSharedPtr<ChBody> chShape2(new ChBody());
	//	chShape2->GetCollisionModel()->ClearModel();
	//	xPos = (lx * kNseg + wallPosx) + boxSx / 2;
	//	//zPos = robotCom.z + zOffset + (pegSeparation / 2 + boxSz / 2 + boxSx/2);  // use this for double wall
	////
	//    zPos = robotCom.z + zOffset + (midBoxSz / 2 + boxSx / 2 + pegSeparation + boxSx / 2 + boxSz / 2);  // use this for triple wall

	//	////////box 2 code ///////////////////
	//	chShape2->GetCollisionModel()->AddBox(boxSx / 2, 1, boxSz / 2, ChVector<>(0, 0, 0));  // half-lengths of box sides
	//	ChSharedPtr<ChBoxShape> boxShape2(new ChBoxShape());
	//	boxShape2->GetBoxGeometry().SetLengths(ChVector<>(boxSx, 2, boxSz)); // full length of box sides
	//	chShape2->AddAsset(boxShape2);

	//	//// add a cylinder
	//	chShape2->GetCollisionModel()->AddCylinder(boxSx / 2, boxSx / 2, 2, ChVector<>(0, 0, -boxSz / 2));  // radius x, radiusz, height on y
	//	ChSharedPtr<ChCylinderShape> cylShape3(new ChCylinderShape());
	//	cylShape3->GetCylinderGeometry().p1 = ChVector<>(0, 1, -boxSz / 2); //vector is position of top cap
	//	cylShape3->GetCylinderGeometry().p2 = ChVector<>(0, -1, -boxSz / 2);   //vector is position of bottom cap
	//	cylShape3->GetCylinderGeometry().rad = boxSx / 2;
	//	chShape2->AddAsset(cylShape3);

	//	chShape2->GetCollisionModel()->AddCylinder(boxSx / 2, boxSx / 2, 2, ChVector<>(0, 0, boxSz / 2));  // radius x, radiusz, height on y
	//	ChSharedPtr<ChCylinderShape> cylShape4(new ChCylinderShape());
	//	cylShape4->GetCylinderGeometry().p1 = ChVector<>(0, 1, boxSz / 2); //vector is position of top cap
	//	cylShape4->GetCylinderGeometry().p2 = ChVector<>(0, -1, boxSz / 2);   //vector is position of bottom cap
	//	cylShape4->GetCylinderGeometry().rad = boxSx / 2;
	//	chShape2->AddAsset(cylShape4);

	//	chShape2->SetBodyFixed(true);
	//	chShape2->SetIdentifier(-3);
	//	chShape2->GetMaterialSurface()->SetFriction(pegFriction);
	//	chShape2->SetPos(ChVector<>(xPos, 0, zPos));
	//	chShape2->SetCollide(true);
	//	chShape2->GetCollisionModel()->BuildModel();
	//	mChSys->AddBody(chShape2);
	//	mCollisionObjs.push_back(chShape2.get());

	////	// for triple wall (double slit), add center segment

	//	ChSharedPtr<ChBody> chShape3(new ChBody());
	//	chShape3->GetCollisionModel()->ClearModel();
	//	xPos = (lx * kNseg + wallPosx) + boxSx / 2;
	//	zPos = robotCom.z + zOffset;

	////	//////box 3 code ///////////////////
	//	chShape3->GetCollisionModel()->AddBox(boxSx / 2, 1, midBoxSz / 2, ChVector<>(0, 0, 0));  // half-lengths of box sides
	//	ChSharedPtr<ChBoxShape> boxShape3(new ChBoxShape());
	//	boxShape3->GetBoxGeometry().SetLengths(ChVector<>(boxSx, 2, midBoxSz)); // full length of box sides
	//	chShape3->AddAsset(boxShape3);

	//	//// add a cylinder
	//	chShape3->GetCollisionModel()->AddCylinder(boxSx / 2, boxSx / 2, 2, ChVector<>(0, 0, -midBoxSz / 2));  // radius x, radiusz, height on y
	//	ChSharedPtr<ChCylinderShape> cylShape5(new ChCylinderShape());
	//	cylShape5->GetCylinderGeometry().p1 = ChVector<>(0, 1, -midBoxSz / 2); //vector is position of top cap
	//	cylShape5->GetCylinderGeometry().p2 = ChVector<>(0, -1, -midBoxSz / 2);   //vector is position of bottom cap
	//	cylShape5->GetCylinderGeometry().rad = boxSx / 2;
	//	chShape3->AddAsset(cylShape5);

	//	chShape3->GetCollisionModel()->AddCylinder(boxSx / 2, boxSx / 2, 2, ChVector<>(0, 0, midBoxSz / 2));  // radius x, radiusz, height on y
	//	ChSharedPtr<ChCylinderShape> cylShape6(new ChCylinderShape());
	//	cylShape6->GetCylinderGeometry().p1 = ChVector<>(0, 1, midBoxSz / 2); //vector is position of top cap
	//	cylShape6->GetCylinderGeometry().p2 = ChVector<>(0, -1, midBoxSz / 2);   //vector is position of bottom cap
	//	cylShape6->GetCylinderGeometry().rad = boxSx / 2;
	//	chShape3->AddAsset(cylShape6);

	//	chShape3->SetBodyFixed(true);
	//	chShape3->SetIdentifier(-4);
	//	chShape3->GetMaterialSurface()->SetFriction(pegFriction);
	//	chShape3->SetPos(ChVector<>(xPos, 0, zPos));
	//	chShape3->SetCollide(true);
	//	chShape3->GetCollisionModel()->BuildModel();
	//	mChSys->AddBody(chShape3);
	//	mCollisionObjs.push_back(chShape3.get());

	//}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// feifei 3 peg setup

	//double zPos = robotCom.z - pegSeparation / 2 - boxSz / 2 - boxSx / 2;
	//double xPos = (robotCom.x + wallPosx) + boxSx / 2;

	//// 1st wall end (right)
	//zPos = robotCom.z - pegSeparation / 2 - boxSx / 2;
	//xPos = (robotCom.x + wallPosx) + boxSx / 2;
	//ChSharedPtr<ChBodyEasyCylinder> chShape3 (new ChBodyEasyCylinder(boxSx/2, boxSy, 1, true, true));
	//chShape3->SetCollide(true);
	//chShape3->SetIdentifier(-1);
	//chShape3->SetFriction(0.0);
	//chShape3->SetPos(ChVector<>(xPos, 0, zPos));
	//chShape3->SetBodyFixed(true);
	//mChSys->AddBody(chShape3);
	//mCollisionObjs.push_back(chShape3.get());

	//
	//// 2nd wall end (middle)
	//zPos = robotCom.z;
	//xPos = (robotCom.x + wallPosx) + boxSx / 2;
	//ChSharedPtr<ChBodyEasyCylinder> chShape5(new ChBodyEasyCylinder(boxSx / 2, boxSy, 1, true, true));
	//chShape5->SetCollide(true);
	//chShape5->SetIdentifier(-1);
	//chShape5->SetFriction(0.0);
	//chShape5->SetPos(ChVector<>(xPos, 0, zPos));
	//chShape5->SetBodyFixed(true);
	//mChSys->AddBody(chShape5);
	//mCollisionObjs.push_back(chShape5.get());
	//
	//// 3rd wall end (left)
	//zPos = robotCom.z + pegSeparation / 2 + boxSx / 2;
	//xPos = (robotCom.x + wallPosx) + boxSx / 2;
	//ChSharedPtr<ChBodyEasyCylinder> chShape4(new ChBodyEasyCylinder(boxSx / 2, boxSy, 1, true, true));
	//chShape4->SetCollide(true);
	//chShape4->SetIdentifier(-1);
	//chShape4->SetFriction(0.0);
	//chShape4->SetPos(ChVector<>(xPos, 0, zPos));
	//chShape4->SetBodyFixed(true);
	//mChSys->AddBody(chShape4);
	//mCollisionObjs.push_back(chShape4.get());
	//

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//feifei
	// 6 peg setup
	//  double zPos;
	// double xPos;

	//  //   peg 1 (farmost left)
	//    zPos = robotCom.z + pegSeparation * 5 / 2 + boxSx * 5 / 2;
	//    xPos = (robotCom.x + wallPosx) + boxSx / 2;
	//    ChSharedPtr<ChBodyEasyCylinder> chShape1(new ChBodyEasyCylinder(boxSx / 2, boxSy, 1, true, true));
	//    chShape1->SetCollide(true);
	//    chShape1->SetIdentifier(-2);
	//    chShape1->SetFriction(0.0);
	//    chShape1->SetPos(ChVector<>(xPos, 0, zPos));
	//    chShape1->SetBodyFixed(true);
	//    mChSys->AddBody(chShape1);
	//    mCollisionObjs.push_back(chShape1.get());

	//  //   peg 2 (2nd left)
	//    zPos = robotCom.z + pegSeparation * 3 / 2 + boxSx * 3 / 2;
	//    xPos = (robotCom.x + wallPosx) + boxSx / 2;
	//    ChSharedPtr<ChBodyEasyCylinder> chShape2(new ChBodyEasyCylinder(boxSx / 2, boxSy, 1, true, true));
	//    chShape2->SetCollide(true);
	//    chShape2->SetIdentifier(-3);
	//    chShape2->SetFriction(0.0);
	//    chShape2->SetPos(ChVector<>(xPos, 0, zPos));
	//    chShape2->SetBodyFixed(true);
	//    mChSys->AddBody(chShape2);
	//    mCollisionObjs.push_back(chShape2.get());

	////     peg 3 (center left)
	//    zPos = robotCom.z + pegSeparation / 2 + boxSx / 2;
	//    xPos = (robotCom.x + wallPosx) + boxSx / 2;
	//    ChSharedPtr<ChBodyEasyCylinder> chShape3(new ChBodyEasyCylinder(boxSx / 2, boxSy, 1, true, true));
	//    chShape3->SetCollide(true);
	//    chShape3->SetIdentifier(-4);
	//    chShape3->SetFriction(0.0);
	//    chShape3->SetPos(ChVector<>(xPos, 0, zPos));
	//    chShape3->SetBodyFixed(true);
	//    mChSys->AddBody(chShape3);
	//    mCollisionObjs.push_back(chShape3.get());

	// //    peg 4 (center right)
	//    zPos = robotCom.z - pegSeparation / 2 - boxSx / 2;
	//    xPos = (robotCom.x + wallPosx) + boxSx / 2;
	//    ChSharedPtr<ChBodyEasyCylinder> chShape4(new ChBodyEasyCylinder(boxSx / 2, boxSy, 1, true, true));
	//    chShape4->SetCollide(true);
	//    chShape4->SetIdentifier(-5);
	//    chShape4->SetFriction(0.0);
	//    chShape4->SetPos(ChVector<>(xPos, 0, zPos));
	//    chShape4->SetBodyFixed(true);
	//    mChSys->AddBody(chShape4);
	//    mCollisionObjs.push_back(chShape4.get());

	////     peg 5 (second right)
	//    zPos = robotCom.z - pegSeparation * 3 / 2 - boxSx * 3 / 2;
	//    xPos = (robotCom.x + wallPosx) + boxSx / 2;
	//    ChSharedPtr<ChBodyEasyCylinder> chShape5(new ChBodyEasyCylinder(boxSx / 2, boxSy, 1, true, true));
	//    chShape5->SetCollide(true);
	//    chShape5->SetIdentifier(-6);
	//    chShape5->SetFriction(0.0);
	//    chShape5->SetPos(ChVector<>(xPos, 0, zPos));
	//    chShape5->SetBodyFixed(true);
	//    mChSys->AddBody(chShape5);
	//    mCollisionObjs.push_back(chShape5.get());

	////     peg 6 (farmost right)
	//    zPos = robotCom.z - pegSeparation * 5 / 2 - boxSx * 5 / 2;
	//    xPos = (robotCom.x + wallPosx) + boxSx / 2;
	//    ChSharedPtr<ChBodyEasyCylinder> chShape6(new ChBodyEasyCylinder(boxSx / 2, boxSy, 1, true, true));
	//    chShape6->SetCollide(true);
	//    chShape6->SetIdentifier(-7);
	//    chShape6->SetFriction(0.0);
	//    chShape6->SetPos(ChVector<>(xPos, 0, zPos));
	//    chShape6->SetBodyFixed(true);
	//    mChSys->AddBody(chShape6);
	//    mCollisionObjs.push_back(chShape6.get());

	//jennifer 4 peg setup
	//double zPos;
	//double xPos;
	////   peg 1 (2nd left)
	//zPos = robotCom.z + pegSeparation * 3 / 2 + boxSx * 3 / 2;
	////xPos = (robotCom.x + wallPosx) + boxSx / 2;
	//xPos = (lx * kNseg + wallPosx) + boxSx / 2;
	//ChSharedPtr<ChBodyEasyCylinder> chShape2(new ChBodyEasyCylinder(boxSx / 2, boxSy, 1, true, true));
	//chShape2->SetCollide(true);
	//chShape2->SetIdentifier(-2);
	//chShape2->SetFriction(pegFriction);
	//chShape2->SetPos(ChVector<>(xPos, 0, zPos));
	//chShape2->SetBodyFixed(true);
	//mChSys->AddBody(chShape2);
	//mCollisionObjs.push_back(chShape2.get());

	////     peg 2 (center left)
	//zPos = robotCom.z + pegSeparation / 2 + boxSx / 2;
	//xPos = (lx * kNseg + wallPosx) + boxSx / 2;
	//ChSharedPtr<ChBodyEasyCylinder> chShape3(new ChBodyEasyCylinder(boxSx / 2, boxSy, 1, true, true));
	//chShape3->SetCollide(true);
	//chShape3->SetIdentifier(-3);
	//chShape3->SetFriction(pegFriction);
	//chShape3->SetPos(ChVector<>(xPos, 0, zPos));
	//chShape3->SetBodyFixed(true);
	//mChSys->AddBody(chShape3);
	//mCollisionObjs.push_back(chShape3.get());

	////    peg 3 (center right)
	//zPos = robotCom.z - pegSeparation / 2 - boxSx / 2;
	//xPos = (lx * kNseg + wallPosx) + boxSx / 2;
	//ChSharedPtr<ChBodyEasyCylinder> chShape4(new ChBodyEasyCylinder(boxSx / 2, boxSy, 1, true, true));
	//chShape4->SetCollide(true);
	//chShape4->SetIdentifier(-4);
	//chShape4->SetFriction(pegFriction);
	//chShape4->SetPos(ChVector<>(xPos, 0, zPos));
	//chShape4->SetBodyFixed(true);
	//mChSys->AddBody(chShape4);
	//mCollisionObjs.push_back(chShape4.get());

	////     peg 4 (second right)
	//zPos = robotCom.z - pegSeparation * 3 / 2 - boxSx * 3 / 2;
	//xPos = (lx * kNseg + wallPosx) + boxSx / 2;
	//ChSharedPtr<ChBodyEasyCylinder> chShape5(new ChBodyEasyCylinder(boxSx / 2, boxSy, 1, true, true));
	//chShape5->SetCollide(true);
	//chShape5->SetIdentifier(-5);
	//chShape5->SetFriction(pegFriction);
	//chShape5->SetPos(ChVector<>(xPos, 0, zPos));
	//chShape5->SetBodyFixed(true);
	//mChSys->AddBody(chShape5);
	//mCollisionObjs.push_back(chShape5.get());

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//////// jennifer: attempting to have number of pegs as an input
	double zPos;
	//double xPos = wallPosx;
	double xPos = (lx * kNseg + wallPosx) + boxSx / 2;

	double num;
	for (int j = 0; j < kNpeg; j++) {

		num = -kNpeg + 2 * j + 1;
		//zPos = robotCom.z + num * (pegSeparation / 2 + boxSx / 2);
		zPos = robotCom.z + zOffset + num * (pegSeparation / 2 + boxSx / 2);

		////         //feifei
		////         //   double zPos = (j - kNpeg / 2) * pegSeparation;
		////         //   ChSharedPtr<ChBodyEasyCylinder> chShape(new ChBodyEasyCylinder(0.05, 3, 1, true, true));

		////            // jennifer: cylinders
#ifdef USE_DEM
		auto chShape = std::make_shared<ChBodyEasyCylinder>(boxSx / 2, boxSy, 1, true, true, ChMaterialSurfaceBase::DEM);
#else
		auto chShape = std::make_shared < ChBodyEasyCylinder
				> (boxSx / 2, boxSy, 1, true, true, ChMaterialSurfaceBase::DVI);
#endif
		////   //

		////////////////////////////////////////////////////////////////////////////////////////////////////////
		//       //  // jennifer  (with Will's help): rounded flat posts
		//       //ChSharedPtr<ChBody> chShape(new ChBody());
		//       //chShape->GetCollisionModel()->ClearModel();
		//       //
		//
		//       //double r = boxSx/10;
		//       //double innerBoxSx = boxSx - 2 * r;
		//       //double cylShift = (boxSx / 2 - r);

		//       ////////box 1 code ///////////////////
		//       //chShape->GetCollisionModel()->AddBox(innerBoxSx/2, 1, innerBoxSx/2+r, ChVector<>(0, 0, 0));  // half-lengths of box sides
		//       //ChSharedPtr<ChBoxShape> boxShape1(new ChBoxShape());
		//       //boxShape1->GetBoxGeometry().SetLengths(ChVector<>(innerBoxSx, 2, innerBoxSx + 2 * r)); // full length of box sides
		//       //chShape->AddAsset(boxShape1);

		//       ////////////box 2 code ///////////////////
		//       //chShape->GetCollisionModel()->AddBox(innerBoxSx/2+r, 1, innerBoxSx/2, ChVector<>(0, 0, 0));
		//       //ChSharedPtr<ChBoxShape> boxShape2(new ChBoxShape());
		//       //boxShape2->GetBoxGeometry().SetLengths(ChVector<>(innerBoxSx + 2 * r, 2, innerBoxSx));
		//       //chShape->AddAsset(boxShape2);

		//       //////////cylinder 1 code ///////////////////
		//       //// // upper left
		//       //chShape->GetCollisionModel()->AddCylinder(r, r, 2, ChVector<>(cylShift, 0, cylShift));  // radius x, radiusz, height on y
		//       //ChSharedPtr<ChCylinderShape> cylShape1(new ChCylinderShape());
		//       //cylShape1->GetCylinderGeometry().p1 = ChVector<>(cylShift, 1, cylShift); //vector is position of top cap
		//       //cylShape1->GetCylinderGeometry().p2 = ChVector<>(cylShift, -1, cylShift);   //vector is position of bottom cap
		//       //cylShape1->GetCylinderGeometry().rad = r;
		//       //chShape->AddAsset(cylShape1);

		//       ////////cylinder 2 code ///////////////////
		//       //// upper right
		//       //chShape->GetCollisionModel()->AddCylinder(r, r, 2, ChVector<>(cylShift, 0, -cylShift));  // radius x, radiusz, height on y
		//       //ChSharedPtr<ChCylinderShape> cylShape2(new ChCylinderShape());
		//       //cylShape2->GetCylinderGeometry().p1 = ChVector<>(cylShift, 1, -cylShift); //vector is position of top cap
		//       //cylShape2->GetCylinderGeometry().p2 = ChVector<>(cylShift, -1, -cylShift);   //vector is position of bottom cap
		//       //cylShape2->GetCylinderGeometry().rad = r;
		//       //chShape->AddAsset(cylShape2);

		//       ////////cylinder 3 code ///////////////////
		//       //// lower left
		//       //chShape->GetCollisionModel()->AddCylinder(r, r, 2, ChVector<>(-cylShift, 0, cylShift));  // radius x, radiusz, height on y
		//       //ChSharedPtr<ChCylinderShape> cylShape3(new ChCylinderShape());
		//       //cylShape3->GetCylinderGeometry().p1 = ChVector<>(-cylShift, 1, cylShift); //vector is position of top cap
		//       //cylShape3->GetCylinderGeometry().p2 = ChVector<>(-cylShift, -1, cylShift);   //vector is position of bottom cap
		//       //cylShape3->GetCylinderGeometry().rad = r;
		//       //chShape->AddAsset(cylShape3);

		//       ////////cylinder 4 code ///////////////////
		//       //// lower right
		//       //chShape->GetCollisionModel()->AddCylinder(r, r, 2, ChVector<>(-cylShift, 0, -cylShift));  // radius x, radiusz, height on y
		//       //ChSharedPtr<ChCylinderShape> cylShape4(new ChCylinderShape());
		//       //cylShape4->GetCylinderGeometry().p1 = ChVector<>(-cylShift, 1, -cylShift); //vector is position of top cap
		//       //cylShape4->GetCylinderGeometry().p2 = ChVector<>(-cylShift, -1, -cylShift);   //vector is position of bottom cap
		//       //cylShape4->GetCylinderGeometry().rad = r;
		//       //chShape->AddAsset(cylShape4);
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		chShape->SetBodyFixed(true);
		chShape->SetIdentifier(-j - 2);
		chShape->SetMaterialSurface(mat_c);
		chShape->SetPos(ChVector<>(xPos, mSnakeParams->h, zPos));
		chShape->SetCollide(true);
		chShape->GetCollisionModel()->BuildModel();

		mChSys->AddBody(chShape);
		mCollisionObjs.push_back(chShape.get());
	}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if (false) {

		for (int i = -1; i <= 1; i += 2) {
			auto chShape = std::make_shared < ChBodyEasyBox
					> (100, 3, 0.2, 1, true, true);
			chShape->SetIdentifier(-1);
			chShape->SetMaterialSurface(mat_c);
			chShape->SetBodyFixed(true);
			chShape->SetPos(ChVector<>(50, 0, i * 1.2));
			mChSys->AddBody(chShape);
		}
	}
}

void ChronoRobotBuilder::BuildBoard_onePeg(double xPos, double zPos, double pegFriction, double boxSx) {
	double boxSy = .2;
#ifdef USE_DEM
	auto mat_c = std::make_shared<ChMaterialSurfaceDEM>();
	mat_c->SetRestitution(dem_cor);
	mat_c->SetYoungModulus(dem_YoungModulus);
	mat_c->SetPoissonRatio(dem_PoissonRatio);
	mat_c->SetFriction(pegFriction);
	mat_c->SetKn(kn);
	mat_c->SetGn(gn);
	auto chShape = std::make_shared<ChBodyEasyCylinder>(boxSx / 2, boxSy, 1, true, true, ChMaterialSurfaceBase::DEM);
#else
	auto mat_c = std::make_shared<ChMaterialSurface>();
	mat_c->SetFriction(pegFriction);
	auto chShape = std::make_shared < ChBodyEasyCylinder
				> (boxSx / 2, boxSy, 1, true, true, ChMaterialSurfaceBase::DVI);
#endif
	chShape->SetBodyFixed(true);
	chShape->SetIdentifier(-2);
	chShape->SetMaterialSurface(mat_c);
	chShape->SetPos(ChVector<>(xPos, mSnakeParams->h, zPos));
	chShape->SetCollide(true);
	chShape->GetCollisionModel()->BuildModel();

	mChSys->AddBody(chShape);
	mCollisionObjs.push_back(chShape.get());
}


void ChronoRobotBuilder::SetCollide(bool mcol) {
	for (int i = 0; i < mCollisionObjs.size(); ++i) {
		mCollisionObjs[i]->SetCollide(mcol);
	}
}

void ChronoRobotBuilder::SetDEM_Coef(double cor, double YoungModulus, double PoissonRatio, double ly) {
	dem_cor = cor;
	dem_YoungModulus = YoungModulus;
	dem_PoissonRatio = PoissonRatio;
	double E_star = dem_YoungModulus / (2.0f * (1 - dem_PoissonRatio * dem_PoissonRatio));
	kn = 0.25f * CH_C_PI * E_star * ly;
	gn = 0;// 1e7;// 1e6;
}
