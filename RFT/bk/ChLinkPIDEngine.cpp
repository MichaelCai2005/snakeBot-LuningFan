//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChLinkPIDEngine.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLinkPIDEngine.h"
#include "physics/ChSystem.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.
#include <algorithm>
namespace chrono
{


    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    //
    //   CLASS FOR ENGINE LINKS
    //
    //

    // Register into the object factory, to enable run-time
    // dynamic creation and persistence
    ChClassRegister<ChLinkPIDEngine> a_registration_ChLinkPIDEngine;


    ChLinkPIDEngine::ChLinkPIDEngine()
    {
        type = LNK_PID_ENGINE;      // initializes type

        rot_funct = new ChFunction_Const(0);
        spe_funct = new ChFunction_Const(0);
        tor_funct = new ChFunction_Const(0);
        torque_w = new ChFunction_Const(1);

        rot_funct_x = new ChFunction_Const(0);
        rot_funct_y = new ChFunction_Const(0);

        mot_rot = mot_rot_dt = mot_rot_dtdt = 0.0;
        mot_rerot = mot_rerot_dt = mot_rerot_dtdt = 0.0;
        mot_torque = mot_retorque = 0.0;
        last_r3mot_rot = 0;
        last_r3mot_rot_dt = 0;
        last_r3relm_rot = QUNIT;
        last_r3relm_rot_dt = QNULL;
        last_r3time = 0;
        keyed_polar_rotation = QNULL;
        impose_reducer = FALSE;

        mot_tau = 1.0;
        mot_eta = 1.0;
        mot_inertia = 0.0;

        cache_li_speed1 = 0;
        cache_li_pos1 = 0;
        torque_react1 = 0;
        cache_li_speed2 = 0;
        cache_li_pos2 = 0;
        torque_react2 = 0;

        eng_mode = ENG_MODE_ROTATION;
        learn = FALSE;


        impose_torque_limit = false;
        torque_limit_max = 0;
        torque_limit_min = 0;

        // Mask: initialize our LinkMaskLF (lock formulation mask)
        // to E3 only.
        ((ChLinkMaskLF*)mask)->SetLockMask(true, false, false,
            false, false, false, true,
            false, false);
        ChangedLinkMask();
        // Mask: initialize remaining LinkMaskLF (lock formulation mask) for the engine.
        // All shaft modes at least are setting the lock on E3 (z-rotation) coordinate.
        Set_shaft_mode(ENG_SHAFT_LOCK);
    }

    ChLinkPIDEngine::~ChLinkPIDEngine()
    {
        if (rot_funct) delete rot_funct;
        if (spe_funct) delete spe_funct;
        if (tor_funct) delete tor_funct;
        if (torque_w)  delete torque_w;

        if (rot_funct_x) delete rot_funct_x;
        if (rot_funct_y) delete rot_funct_y;
    }

    void ChLinkPIDEngine::Copy(ChLinkPIDEngine* source)
    {
        // first copy the parent class data...
        //
        ChLinkLock::Copy(source);

        // copy custom data:
        learn = source->learn;
        eng_mode = source->eng_mode;
        shaft_mode = source->shaft_mode;

        mot_rot = source->mot_rot;
        mot_rot_dt = source->mot_rot_dt;
        mot_rot_dtdt = source->mot_rot_dtdt;
        mot_rerot = source->mot_rerot;
        mot_rerot_dt = source->mot_rerot_dt;
        mot_rerot_dtdt = source->mot_rerot_dtdt;
        mot_torque = source->mot_torque;
        mot_retorque = source->mot_retorque;
        impose_reducer = source->impose_reducer;
        last_r3time = source->last_r3time;
        last_r3mot_rot = source->last_r3mot_rot;
        last_r3mot_rot_dt = source->last_r3mot_rot_dt;
        last_r3relm_rot = source->last_r3relm_rot;
        last_r3relm_rot_dt = source->last_r3relm_rot_dt;
        keyed_polar_rotation = source->keyed_polar_rotation;

        if (rot_funct) delete rot_funct;
        rot_funct = source->rot_funct->new_Duplicate();
        if (spe_funct) delete spe_funct;
        spe_funct = source->spe_funct->new_Duplicate();
        if (tor_funct) delete tor_funct;
        tor_funct = source->tor_funct->new_Duplicate();
        if (torque_w)  delete torque_w;
        torque_w = source->torque_w->new_Duplicate();

        if (rot_funct_x) delete rot_funct_x;
        rot_funct_x = source->rot_funct_x->new_Duplicate();
        if (rot_funct_y) delete rot_funct_y;
        rot_funct_y = source->rot_funct_y->new_Duplicate();

        mot_tau = source->mot_tau;
        mot_eta = source->mot_eta;
        mot_inertia = source->mot_inertia;

        cache_li_speed1 = 0;
        cache_li_pos1 = 0;
        torque_react1 = source->torque_react1;
        cache_li_speed2 = 0;
        cache_li_pos2 = 0;
        torque_react2 = source->torque_react2;


        impose_torque_limit = source->impose_torque_limit;
        torque_limit_max = source->torque_limit_max;
        torque_limit_min = source->torque_limit_min;
    }

    ChLink* ChLinkPIDEngine::new_Duplicate()
    {
        ChLinkPIDEngine* m_l;
        m_l = new ChLinkPIDEngine;  // inherited classes should write here: m_l = new MyInheritedLink;
        m_l->Copy(this);
        return (m_l);
    }

    void ChLinkPIDEngine::Set_learn(int mset)
    {
        learn = mset;

        if ((eng_mode == ENG_MODE_ROTATION) ||
            (eng_mode == ENG_MODE_SPEED) ||
            (eng_mode == ENG_MODE_KEY_ROTATION))
        {
            if (mset)
                ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_FREE);
            else
                ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_LOCK);

            ChangedLinkMask();
        }

        if (eng_mode == ENG_MODE_KEY_POLAR)
        {
            if (mset) {
                ((ChLinkMaskLF*)mask)->Constr_E1().SetMode(CONSTRAINT_FREE);
                ((ChLinkMaskLF*)mask)->Constr_E2().SetMode(CONSTRAINT_FREE);
                ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_FREE);
            }
            else {
                ((ChLinkMaskLF*)mask)->Constr_E1().SetMode(CONSTRAINT_LOCK);
                ((ChLinkMaskLF*)mask)->Constr_E2().SetMode(CONSTRAINT_LOCK);
                ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_LOCK);
            }
            ChangedLinkMask();
        }

        if (eng_mode == ENG_MODE_ROTATION)
        if (rot_funct->Get_Type() != FUNCT_RECORDER) // if wasn't recorder f()..
        {
            delete (rot_funct); rot_funct = NULL;
            rot_funct = new ChFunction_Recorder;
        }

        if (eng_mode == ENG_MODE_SPEED)
        if (spe_funct->Get_Type() != FUNCT_RECORDER) // if wasn't recorder f()..
        {
            delete (spe_funct); spe_funct = NULL;
            spe_funct = new ChFunction_Recorder;
        }
    }

    void ChLinkPIDEngine::Set_rot_funct(ChFunction* m_funct)
    {
        if (rot_funct) delete rot_funct;
        rot_funct = m_funct;
    }
    void ChLinkPIDEngine::Set_spe_funct(ChFunction* m_funct)
    {
        if (spe_funct) delete spe_funct;
        spe_funct = m_funct;
    }
    void ChLinkPIDEngine::Set_tor_funct(ChFunction* m_funct)
    {
        if (tor_funct) delete tor_funct;
        tor_funct = m_funct;
    }
    void ChLinkPIDEngine::Set_torque_w_funct(ChFunction* m_funct)
    {
        if (torque_w) delete torque_w;
        torque_w = m_funct;
    }
    void ChLinkPIDEngine::Set_rot_funct_x(ChFunction* m_funct_x)
    {
        if (rot_funct_x) delete rot_funct_x;
        rot_funct_x = m_funct_x;
    }
    void ChLinkPIDEngine::Set_rot_funct_y(ChFunction* m_funct_y)
    {
        if (rot_funct_y) delete rot_funct_y;
        rot_funct_y = m_funct_y;
    }
    void ChLinkPIDEngine::SetKeyedPolarRotation(Quaternion mq)
    {
        keyed_polar_rotation = mq;
    }


    void ChLinkPIDEngine::Set_eng_mode(int mset)
    {
        if (Get_learn()) Set_learn(FALSE); // reset learn state when changing mode

        if (eng_mode != mset)
        {
            eng_mode = mset;
            if (eng_mode == ENG_MODE_ROTATION ||
                eng_mode == ENG_MODE_SPEED ||
                eng_mode == ENG_MODE_KEY_ROTATION)
            {
                ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_LOCK);
            }

            if (eng_mode == ENG_MODE_KEY_POLAR)
            {
                ((ChLinkMaskLF*)mask)->Constr_E1().SetMode(CONSTRAINT_LOCK);
                ((ChLinkMaskLF*)mask)->Constr_E2().SetMode(CONSTRAINT_LOCK);
                ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_LOCK);
            }

            if (eng_mode == ENG_MODE_TORQUE)
            {
                ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_FREE);

            }

            if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
            {
                ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_FREE);
            }
            ChangedLinkMask();  // update all from new mask
        }

        if (eng_mode == ENG_MODE_KEY_ROTATION)
        if (rot_funct->Get_Type() != FUNCT_CONST) // if wasn't constant f()..
        {
            delete (rot_funct); rot_funct = NULL;
            rot_funct = new ChFunction_Const;
        }

        if (eng_mode == ENG_MODE_KEY_POLAR)
        {
            if (rot_funct->Get_Type() != FUNCT_CONST)  {
                delete (rot_funct); rot_funct = NULL;
                rot_funct = new ChFunction_Const;
            }
            if (rot_funct_x->Get_Type() != FUNCT_CONST)  {
                delete (rot_funct_x); rot_funct_x = NULL;
                rot_funct_x = new ChFunction_Const;
            }
            if (rot_funct_y->Get_Type() != FUNCT_CONST)  {
                delete (rot_funct_y); rot_funct_y = NULL;
                rot_funct_y = new ChFunction_Const;
            }
        }
    }

    void ChLinkPIDEngine::Set_shaft_mode(int mset)
    {
        shaft_mode = mset;

        eChConstraintMode curr_mode_z = ((ChLinkMaskLF*)mask)->Constr_E3().GetMode();

        switch (shaft_mode)
        {
        case ENG_SHAFT_PRISM:
            ((ChLinkMaskLF*)mask)->SetLockMask(true, true, false,
                false, true, true, true, // <-
                false, false); break;
        case ENG_SHAFT_UNIVERSAL:
            ((ChLinkMaskLF*)mask)->SetLockMask(true, true, true,
                false, false, false, true, // <-
                false, false); break;
        case ENG_SHAFT_CARDANO:
            ((ChLinkMaskLF*)mask)->SetLockMask(false, false, false,
                false, false, false, true, // <-
                false, false); break;
        case ENG_SHAFT_OLDHAM:
            ((ChLinkMaskLF*)mask)->SetLockMask(false, false, false,
                false, true, true, true, // <-
                false, false); break;
        case ENG_SHAFT_LOCK:
        default:
            ((ChLinkMaskLF*)mask)->SetLockMask(true, true, true,
                false, true, true, true, // <-
                false, false); break;
        }

        ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(curr_mode_z);


        // change datas
        ChangedLinkMask();
    }


    void ChLinkPIDEngine::UpdatedExternalTime(double prevtime, double time)
    {
        last_r3time = ChTime;
        last_r3mot_rot = Get_mot_rot();
        last_r3mot_rot_dt = Get_mot_rot_dt();
        last_r3relm_rot = GetRelM().rot;
        last_r3relm_rot_dt = GetRelM_dt().rot;
    }

    // a cosine ramp function
    // A * (1 - cos(omega * (t2 - t1)))
    double rampcosine(double t1, double val1, double t2, double val2)
    {
        if (t2 < t1)
            return val1;
        double omega = CH_C_2PI;
        double amp = val2 - val1;
        double tp = amp / omega;
        if (t2 - t1 > tp)
            return val2;
        return amp * (1 - cos(omega * (t2 - t1))) + val1;
    }

    void ChLinkPIDEngine::UpdateTime(double mytime)
    {
        // First, inherit to parent class
        ChLinkLock::UpdateTime(mytime);

        if (!IsActive())
            return;

        // DEFAULTS compute rotation vars...
        // by default for torque control..

        motion_axis = VECT_Z;       // motion axis is always the marker2 Z axis (in m2 relative coords)
        mot_rot = relAngle;
        mot_rot_dt = Vdot(relWvel, motion_axis);
        mot_rot_dtdt = Vdot(relWacc, motion_axis);
        mot_rerot = mot_rot / mot_tau;
        mot_rerot_dt = mot_rot_dt / mot_tau;
        mot_rerot_dtdt = mot_rot_dtdt / mot_tau;
        // nothing more to do here for torque control
        return;


        // If LEARN MODE, just record motion
        if (learn == TRUE)
        {
            deltaC.pos = VNULL;
            deltaC_dt.pos = VNULL;
            deltaC_dtdt.pos = VNULL;
            if (!(limit_Rx->Get_active() ||
                limit_Ry->Get_active() ||
                limit_Rz->Get_active()))
            {
                deltaC.rot = QUNIT;
                deltaC_dt.rot = QNULL;
                deltaC_dtdt.rot = QNULL;
            }

            if (eng_mode == ENG_MODE_ROTATION)
            {
                if (rot_funct->Get_Type() != FUNCT_RECORDER) // if wasn't recorder f()..
                {
                    delete (rot_funct); rot_funct = NULL;
                    rot_funct = new ChFunction_Recorder;
                }
                // record point
                double rec_rot = relAngle; // ***TO DO*** compute also rotations with cardano mode?
                if (impose_reducer)
                    rec_rot = rec_rot / mot_tau;
                ((ChFunction_Recorder*)rot_funct)->AddPoint(mytime, rec_rot, 1);  //  x=t
            }
            if (eng_mode == ENG_MODE_SPEED)
            {
                if (spe_funct->Get_Type() != FUNCT_RECORDER) // if wasn't recorder f()..
                {
                    delete (spe_funct); spe_funct = NULL;
                    spe_funct = new ChFunction_Recorder;
                }
                // record point
                double rec_spe = relWvel.Length(); // ***TO DO*** compute also with cardano mode?
                if (impose_reducer)
                    rec_spe = rec_spe / mot_tau;
                ((ChFunction_Recorder*)spe_funct)->AddPoint(mytime, rec_spe, 1);  //  x=t
            }
            return;
        }

        

        // Impose relative positions/speeds

        

        if (eng_mode == ENG_MODE_ROTATION)
        {
            deltaC.pos = VNULL;
            deltaC_dt.pos = VNULL;
            deltaC_dtdt.pos = VNULL;
            if (impose_reducer)
            {
                mot_rerot = rot_funct->Get_y(ChTime);
                mot_rerot_dt = rot_funct->Get_y_dx(ChTime);
                mot_rerot_dtdt = rot_funct->Get_y_dxdx(ChTime);
                mot_rot = mot_rerot * mot_tau;
                mot_rot_dt = mot_rerot_dt * mot_tau;
                mot_rot_dtdt = mot_rerot_dtdt * mot_tau;
            }
            else {
                mot_rot = rot_funct->Get_y(ChTime);
                mot_rot_dt = rot_funct->Get_y_dx(ChTime);
                mot_rot_dtdt = rot_funct->Get_y_dxdx(ChTime);
                mot_rerot = mot_rot / mot_tau;
                mot_rerot_dt = mot_rot_dt / mot_tau;
                mot_rerot_dtdt = mot_rot_dtdt / mot_tau;
            }

            deltaC.rot = Q_from_AngAxis(mot_rot, motion_axis);
            deltaC_dt.rot = Qdt_from_AngAxis(deltaC.rot, mot_rot_dt, motion_axis);
            deltaC_dtdt.rot = Qdtdt_from_AngAxis(mot_rot_dtdt, motion_axis, deltaC.rot, deltaC_dt.rot);
        }
        if (eng_mode == ENG_MODE_SPEED)
        {
            deltaC.pos = VNULL;
            deltaC_dt.pos = VNULL;
            deltaC_dtdt.pos = VNULL;
            if (impose_reducer)
            {
                mot_rerot_dt = spe_funct->Get_y(ChTime);
                mot_rerot_dtdt = spe_funct->Get_y_dx(ChTime);
                mot_rot_dt = mot_rerot_dt * mot_tau;
                mot_rot_dtdt = mot_rerot_dtdt * mot_tau;
            }
            else {
                mot_rot_dt = spe_funct->Get_y(ChTime);
                mot_rot_dtdt = spe_funct->Get_y_dx(ChTime);
                mot_rerot_dt = mot_rot_dt / mot_tau;
                mot_rerot_dtdt = mot_rot_dtdt / mot_tau;
            }
            
            deltaC.rot = Qnorm(GetRelM().rot);    // just keep current position, -assume always good after integration-.
            ChMatrix33<> relA; relA.Set_A_quaternion(GetRelM().rot);  // ..but adjust to keep Z axis aligned to shaft, anyway!
            ChVector<> displaced_z_axis; displaced_z_axis = relA.Get_A_Zaxis();
            ChVector<> adjustment = Vcross(displaced_z_axis, VECT_Z);
            deltaC.rot = Q_from_AngAxis(adjustment.Length(), Vnorm(adjustment)) % deltaC.rot;
            deltaC_dt.rot = Qdt_from_AngAxis(deltaC.rot, mot_rot_dt, motion_axis);
            deltaC_dtdt.rot = Qdtdt_from_AngAxis(mot_rot_dtdt, motion_axis, deltaC.rot, deltaC_dt.rot);
        }

    }

    void ChLinkPIDEngine::UpdateForces(double mytime)
    {
        // First, inherit to parent class
        ChLinkLock::UpdateForces(mytime);

        if (!IsActive())
            return;

        // DEFAULTS set null torques
        mot_torque = 0;
        mot_retorque = 0;

        if (eng_mode == ENG_MODE_TORQUE)
        {
            // in torque mode, apply the torque vector to both m1 and m2
            //  -  M= f(t)
            double my_torque = Get_tor_funct()->Get_y(ChTime);

            if (impose_reducer)
            {
                my_torque = my_torque*Get_torque_w_funct()->Get_y(mot_rerot_dt);
                mot_retorque = my_torque;
                mot_torque = mot_retorque*(mot_eta / mot_tau);
            }
            else {
                my_torque = my_torque*Get_torque_w_funct()->Get_y(mot_rot_dt);
                mot_torque = my_torque;
                mot_retorque = mot_retorque*(mot_tau / mot_eta);
            }

            if (impose_torque_limit)
            {
                motion_axis = VECT_Z;       // motion axis is always the marker2 Z axis (in m2 relative coords)
                mot_rot = relAngle;
                mot_rot_dt = Vdot(relWvel, motion_axis);
                mot_rot_dtdt = Vdot(relWacc, motion_axis);
                mot_rerot = mot_rot / mot_tau;
                mot_rerot_dt = mot_rot_dt / mot_tau;
                mot_rerot_dtdt = mot_rot_dtdt / mot_tau;
                double my_mot_rerot, my_mot_rot, my_mot_rerot_dt, my_mot_rot_dt;
                if (impose_reducer)
                {
                    my_mot_rerot = rot_funct->Get_y(ChTime);
                    my_mot_rerot_dt = rot_funct->Get_y_dx(ChTime);
                    my_mot_rot = my_mot_rerot * mot_tau;
                    my_mot_rot_dt = my_mot_rerot_dt * mot_tau;
                }
                else {

                    my_mot_rot = rot_funct->Get_y(ChTime);
                    my_mot_rot_dt = rot_funct->Get_y_dx(ChTime);
                    my_mot_rerot = my_mot_rot / mot_tau;
                    my_mot_rerot_dt = my_mot_rot_dt / mot_tau;
                }

                // applied torque
                cum_erno += my_mot_rot - mot_rot;
                double error_rot = my_mot_rot - mot_rot;
                
                if (error_rot > CH_C_PI_2)
                    error_rot = CH_C_PI_2;
                if (error_rot < -CH_C_PI_2)
                    error_rot = -CH_C_PI_2;
                
                double error_ome = my_mot_rot_dt - mot_rot_dt;
                if (error_ome > CH_C_2PI)
                    error_ome = CH_C_2PI;
                if (error_ome < -CH_C_2PI)
                    error_ome = -CH_C_2PI;

                const double tstep = GetSystem()->GetTimerStep();
                double my_torque = (rot_gain * error_rot + int_gain * cum_erno + der_gain * error_ome);

                if (impose_reducer)
                {
                    my_torque = my_torque*Get_torque_w_funct()->Get_y(mot_rerot_dt);
                    mot_retorque = my_torque;
                    mot_torque = mot_retorque*(mot_eta / mot_tau);
                }
                else {
                    my_torque = my_torque*Get_torque_w_funct()->Get_y(mot_rot_dt);
                    mot_torque = my_torque;
                    mot_retorque = mot_retorque*(mot_tau / mot_eta);
                }
                // saturation
                if (mot_torque > torque_limit_max)
                    mot_torque = torque_limit_max;
                if (mot_torque < torque_limit_min)
                    mot_torque = torque_limit_min;

            }
            Vector mv_torque = Vmul(motion_axis, mot_torque);

            // +++ADD TO LINK TORQUE VECTOR
            C_torque = Vadd(C_torque, mv_torque);

        }

        if ((eng_mode == ENG_MODE_ROTATION) ||
            (eng_mode == ENG_MODE_SPEED) ||
            (eng_mode == ENG_MODE_KEY_ROTATION))
        {
            mot_torque = react_torque.z;
            mot_retorque = mot_torque*(mot_tau / mot_eta) + mot_rerot_dtdt*mot_inertia;
        }

        if (eng_mode == ENG_MODE_SPEED)
        {
            // trick: zeroes Z rotat. violation to tell that rot.position is always ok
            if (C->GetRows())
                C->SetElement(C->GetRows() - 1, 0, 0.0);
        }
    }



    void ChLinkPIDEngine::SetUpMarkers(ChMarker* mark1, ChMarker* mark2)
    {
        ChLinkMasked::SetUpMarkers(mark1, mark2);

        if (Body1 && Body2)
        {
            innerconstraint1.SetVariables(&Body1->Variables(), &innershaft1.Variables());
            innerconstraint2.SetVariables(&Body2->Variables(), &innershaft2.Variables());
        }
    }

    //
    //  LCP functions
    //


    void ChLinkPIDEngine::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
    {
        // First, inherit to parent class
        ChLinkLock::InjectConstraints(mdescriptor);

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            mdescriptor.InsertConstraint(&innerconstraint1);
            mdescriptor.InsertConstraint(&innerconstraint2);
        }
    }

    void ChLinkPIDEngine::ConstraintsBiReset()
    {
        // First, inherit to parent class
        ChLinkLock::ConstraintsBiReset();

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            innerconstraint1.Set_b_i(0.);
            innerconstraint2.Set_b_i(0.);
        }
    }

    void ChLinkPIDEngine::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
    {
        // First, inherit to parent class
        ChLinkLock::ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            double res = 0; // no residual
            innerconstraint1.Set_b_i(innerconstraint1.Get_b_i() + factor * res);
            innerconstraint2.Set_b_i(innerconstraint2.Get_b_i() + factor * res);
        }
    }

    void ChLinkPIDEngine::ConstraintsBiLoad_Ct(double factor)
    {
        // First, inherit to parent class
        ChLinkLock::ConstraintsBiLoad_Ct(factor);

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            // nothing
        }
    }

    void ChLinkPIDEngine::ConstraintsLoadJacobians()
    {
        // First, inherit to parent class
        ChLinkLock::ConstraintsLoadJacobians();

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            // compute jacobians
            ChVector<> tempz = ChVector<>(VECT_Z);
            ChVector<> abs_rot_axis = marker2->Dir_Ref2World(&tempz);
            ChVector<> jacw = Body2->TransformDirectionParentToLocal(abs_rot_axis);

            innerconstraint1.Get_Cq_a()->ElementN(0) = 0;
            innerconstraint1.Get_Cq_a()->ElementN(1) = 0;
            innerconstraint1.Get_Cq_a()->ElementN(2) = 0;
            innerconstraint1.Get_Cq_a()->ElementN(3) = (float)jacw.x;
            innerconstraint1.Get_Cq_a()->ElementN(4) = (float)jacw.y;
            innerconstraint1.Get_Cq_a()->ElementN(5) = (float)jacw.z;
            innerconstraint1.Get_Cq_b()->ElementN(0) = -1;

            innerconstraint2.Get_Cq_a()->ElementN(0) = 0;
            innerconstraint2.Get_Cq_a()->ElementN(1) = 0;
            innerconstraint2.Get_Cq_a()->ElementN(2) = 0;
            innerconstraint2.Get_Cq_a()->ElementN(3) = (float)jacw.x;
            innerconstraint2.Get_Cq_a()->ElementN(4) = (float)jacw.y;
            innerconstraint2.Get_Cq_a()->ElementN(5) = (float)jacw.z;
            innerconstraint2.Get_Cq_b()->ElementN(0) = -1;
        }
    }


    void ChLinkPIDEngine::ConstraintsFetch_react(double factor)
    {
        // First, inherit to parent class
        ChLinkLock::ConstraintsFetch_react(factor);

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            // From constraints to react vector:
            torque_react1 = innerconstraint1.Get_l_i() * factor;
            torque_react2 = innerconstraint2.Get_l_i() * factor;
        }
    }

    void  ChLinkPIDEngine::ConstraintsLiLoadSuggestedSpeedSolution()
    {
        // First, inherit to parent class
        ChLinkLock::ConstraintsLiLoadSuggestedSpeedSolution();

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            innerconstraint1.Set_l_i(cache_li_speed1);
            innerconstraint2.Set_l_i(cache_li_speed2);
        }
    }

    void  ChLinkPIDEngine::ConstraintsLiLoadSuggestedPositionSolution()
    {
        // First, inherit to parent class
        ChLinkLock::ConstraintsLiLoadSuggestedPositionSolution();

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            innerconstraint1.Set_l_i(cache_li_pos1);
            innerconstraint2.Set_l_i(cache_li_pos2);
        }
    }

    void  ChLinkPIDEngine::ConstraintsLiFetchSuggestedSpeedSolution()
    {
        // First, inherit to parent class
        ChLinkLock::ConstraintsLiFetchSuggestedSpeedSolution();

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            cache_li_speed1 = (float)innerconstraint1.Get_l_i();
            cache_li_speed2 = (float)innerconstraint2.Get_l_i();
        }
    }

    void  ChLinkPIDEngine::ConstraintsLiFetchSuggestedPositionSolution()
    {
        // First, inherit to parent class
        ChLinkLock::ConstraintsLiFetchSuggestedPositionSolution();

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            cache_li_pos1 = (float)innerconstraint1.Get_l_i();
            cache_li_pos2 = (float)innerconstraint2.Get_l_i();
        }
    }

    void ChLinkPIDEngine::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
    {
        // First, inherit to parent class
        ChLinkLock::InjectVariables(mdescriptor);

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            innershaft1.InjectVariables(mdescriptor);
            innershaft2.InjectVariables(mdescriptor);
        }
    }

    void ChLinkPIDEngine::VariablesFbReset()
    {
        // First, inherit to parent class
        ChLinkLock::VariablesFbReset();

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            innershaft1.VariablesFbReset();
            innershaft2.VariablesFbReset();
        }
    }

    void ChLinkPIDEngine::VariablesFbLoadForces(double factor)
    {
        // First, inherit to parent class
        ChLinkLock::VariablesFbLoadForces(factor);

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            innershaft1.VariablesFbLoadForces(factor);
            innershaft2.VariablesFbLoadForces(factor);
        }
    }

    void ChLinkPIDEngine::VariablesFbIncrementMq()
    {
        // inherit parent class
        ChLinkLock::VariablesFbIncrementMq();

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            innershaft1.VariablesFbIncrementMq();
            innershaft2.VariablesFbIncrementMq();
        }
    }

    void ChLinkPIDEngine::VariablesQbLoadSpeed()
    {
        // First, inherit to parent class
        ChLinkLock::VariablesQbLoadSpeed();

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            innershaft1.VariablesQbLoadSpeed();
            innershaft2.VariablesQbLoadSpeed();
        }
    }

    void ChLinkPIDEngine::VariablesQbSetSpeed(double step)
    {
        // First, inherit to parent class
        ChLinkLock::VariablesQbSetSpeed(step);

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            innershaft1.VariablesQbSetSpeed(step);
            innershaft2.VariablesQbSetSpeed(step);
        }
    }

    void ChLinkPIDEngine::VariablesQbIncrementPosition(double step)
    {
        // First, inherit to parent class
        ChLinkLock::VariablesQbIncrementPosition(step);

        if (eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT)
        {
            innershaft1.VariablesQbIncrementPosition(step);
            innershaft2.VariablesQbIncrementPosition(step);
        }
    }







    void ChLinkPIDEngine::StreamOUT(ChStreamOutBinary& mstream)
    {
        // class version number
        mstream.VersionWrite(1);
        // serialize parent class too
        ChLinkLock::StreamOUT(mstream);

        // stream out all member data
        mstream.AbstractWrite(rot_funct);
        mstream.AbstractWrite(spe_funct);
        mstream.AbstractWrite(tor_funct);
        mstream.AbstractWrite(torque_w);
        mstream << learn;
        mstream << impose_reducer;
        mstream << mot_tau;
        mstream << mot_eta;
        mstream << mot_inertia;
        mstream << eng_mode;
        mstream << shaft_mode;
    }

    void ChLinkPIDEngine::StreamIN(ChStreamInBinary& mstream)
    {
        // class version number
        int version = mstream.VersionRead();
        // deserialize parent class too
        ChLinkLock::StreamIN(mstream);

        // stream in all member data
        ChFunction* ffoo;
        mstream.AbstractReadCreate(&ffoo);		Set_rot_funct(ffoo);
        mstream.AbstractReadCreate(&ffoo);		Set_spe_funct(ffoo);
        mstream.AbstractReadCreate(&ffoo);		Set_tor_funct(ffoo);
        mstream.AbstractReadCreate(&ffoo);		Set_torque_w_funct(ffoo);
        mstream >> learn;
        mstream >> impose_reducer;
        mstream >> mot_tau;
        mstream >> mot_eta;
        mstream >> mot_inertia;
        mstream >> eng_mode;
        mstream >> shaft_mode;
    }

    void ChLinkPIDEngine::Get_mot_torque_limits(double *tormax, double *tormin)
    {
        *tormax = torque_limit_max;
        *tormin = torque_limit_min;
    }
    void ChLinkPIDEngine::Set_mot_torque_limits(double tormax, double tormin)
    {
        torque_limit_max = tormax;
        torque_limit_min = tormin;
        impose_torque_limit = true;
    }

    void ChLinkPIDEngine::Set_mot_gain(double g1, double g2, double g3)
    {
        rot_gain = g1;
        int_gain = g2;
        der_gain = g3;
    }

    void ChLinkPIDEngine::Get_mot_gain(double* g1, double* g2, double* g3)
    {
        *g1 = rot_gain;
        *g2 = int_gain;
        *g3 = der_gain;
    }
    ///////////////////////////////////////////////////////////////



} // END_OF_NAMESPACE____


