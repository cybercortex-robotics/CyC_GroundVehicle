// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CMPCWrapper.h"
#include <iostream>
#include "acado_common.h"
#include "helpers.h"
#include <cmath>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;


void CMPCWrapper::Initialize(float ref_speed)
{
    Eigen::VectorXf weight_vector(3);
    weight_vector(0) = 1.0f;  // ref velocity
    weight_vector(1) = 3.0f;  // epsi
    weight_vector(2) = 10.0f;  // cte
    /* Some temporary variables. */
    int    i;

    /* Initialize the solver. */
    acado_initializeSolver();

    /* Initialize the states and controls. */
    for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[i] = 0.0f;
    for (i = 0; i < NU * N; ++i)  acadoVariables.u[i] = 0.1f;

    /* Initialize the measurements/reference. */
    for (i = 0; i < NY * N; ++i)  acadoVariables.y[i] = 0.0f;
    for (i = 0; i < NYN; ++i)  acadoVariables.yN[i] = 0.0f;

    for (i = 0; i < N; i++) {
        // Setup diagonal entries
        for (int j = 0; j < NYN; ++j)
        {
            acadoVariables.W[NY*NY*i + (NY + 1) * j] = weight_vector[j];
        }
    }

    // Setup final weight matrix
    for (int j = 0; j < NYN; ++j) acadoVariables.WN[(NYN + 1) * j] = weight_vector[j];

    /* MPC: initialize the current state feedback. */
    for (i = 0; i < NX; ++i) acadoVariables.x0[i] = 0.0f;

    SetReferenceVelocity(ref_speed);
}

void CMPCWrapper::SetReferenceVelocity(float v_ref)
{
    for (int i = 0; i < N * NY; i += NY) acadoVariables.y[i] = v_ref;
    acadoVariables.yN[0] = v_ref;
}

bool CMPCWrapper::Solve(const CycState& x0, const Eigen::VectorXf& poly)
{
    // Set initial state
    for (int i = 0; i < NX; ++i) acadoVariables.x0[i] = x0.x_hat[i];

    // Set polynomial fit coefficients
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < NOD - 1; ++j) // Last OD variable is LF
        {
            acadoVariables.od[i * NOD + j] = poly[j];
        }
        acadoVariables.od[i * NOD + (NOD - 1)] = m_VehicleModel.m_fVehicleLength;
    }

    /* Prepare first step */
    if (acado_preparationStep() != 0) return false;
    /* Perform the feedback step. */
    if (acado_feedbackStep() != 0) return false;
    
    return true;
 
}

CycControlInput CMPCWrapper::GetControlSolution()
{
    CycControlInput ctrl_ret;
    ctrl_ret.u = Eigen::VectorXf(NU);
    for (int i = 0; i < NU; ++i) ctrl_ret.u[i] = static_cast<float>(acadoVariables.u[i]);
    return ctrl_ret;
}

CycState CMPCWrapper::GetState()
{
    CycState state;
    state.x_hat = Eigen::VectorXf(NX);
    for (int i = 0; i < NX; ++i) state.x_hat[i] = float(acadoVariables.x[NX + i]);
    return state;
}

Eigen::VectorXf CMPCWrapper::SolveAndCommand(const CycReferenceSetPoints& ref_points, const CycState& vehicle_state)
{
    if (ref_points.ref.size() > 0) {
        CycState state;
        state = vehicle_state;
        std::swap(state.x_hat(2), state.x_hat(3));
        Eigen::VectorXf coeffs = get_poly_coefficients_ordered(ref_points, state, 10);
		state.x_hat(0) = 0.f;
        state.x_hat(1) = 0.f;
        state.x_hat(2) = 0.f;
        bool bSolved = Solve(state, coeffs);
        CycControlInput solution;
        if (bSolved == true) {
            solution = GetControlSolution();
			if (!std::isnan(solution.u(0)) && !std::isnan(solution.u(1)))
			{
				previous_input = solution;
			}
        }
    }

    previous_input.u(0) = IntegrateAcc(previous_input.u(0));
    return previous_input.u;
}
