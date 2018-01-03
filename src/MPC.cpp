#include "MPC.h"
#include "Eigen-3.3/Eigen/Core"
#include "utils.h"
#include "config.h"

using CppAD::AD;


MPC::MPC(double artificialLatency)
    : dt(0.15)
    , Lf(2.67)
    , mStateSize(6U)
    , mNbActuators(2U)
    , mArtificialLatencyMs(artificialLatency)
    , mRuntime(mArtificialLatencyMs)
    , N(8)
    , mTargetSpeed(90)
    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    , actuatorsArraySize(static_cast<size_t>( mNbActuators * ( N - 1 ) ))
    , constraintsArraySize(static_cast<size_t>( N * mStateSize ))
    , nbVars(actuatorsArraySize + constraintsArraySize)
    , mState(nbVars)
    , mState_lowerbound(nbVars)
    , mState_upperbound(nbVars)
    , mState_constraint_lowerbound(constraintsArraySize)
    , mState_constraint_upperbound(constraintsArraySize)
    , mFG_eval(N, dt, Lf, mTargetSpeed)
{
    for (int i = 0; i < nbVars; i++) {
        mState[i] = 0.0;
    }

    //Set lower and upper limits for variables.
    for (int i = 0; i < delta_start; i++) {
        mState_lowerbound[i] = -1.0e19;
        mState_upperbound[i] = 1.0e19;
    }

    // Limit steering delta to stay within [-25 deg, +25 deg] range
    for (int i = delta_start; i < a_start; i++) {
        mState_lowerbound[i] = -0.436332;
        mState_upperbound[i] = 0.436332;
    }

    // Limit to the acceleration to be the maximum possible allowed by the car
    for (int i = a_start; i < nbVars; i++) {
        mState_lowerbound[i] = -1.0;
        mState_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    for (int i = 0; i < constraintsArraySize; i++) {
        mState_constraint_lowerbound[i] = 0;
        mState_constraint_upperbound[i] = 0;
    }
}

MPC::~MPC()
{
}

void MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;

    // I set the initial state
    mState[x_start] = state[0];
    mState[y_start] = state[1];
    mState[psi_start] = state[2];
    mState[v_start] = state[3];
    mState[cte_start] = state[4];
    mState[epsi_start] = state[5];

    // Set the initial state lower limits to current state vector values
    mState_constraint_lowerbound[x_start] = state[0];
    mState_constraint_lowerbound[y_start] = state[1];
    mState_constraint_lowerbound[psi_start] = state[2];
    mState_constraint_lowerbound[v_start] = state[3];
    mState_constraint_lowerbound[cte_start] = state[4];
    mState_constraint_lowerbound[epsi_start] = state[5];
    // Set the initial state upper limits to current state vector values
    mState_constraint_upperbound[x_start] = state[0];
    mState_constraint_upperbound[y_start] = state[1];
    mState_constraint_upperbound[psi_start] = state[2];
    mState_constraint_upperbound[v_start] = state[3];
    mState_constraint_upperbound[cte_start] = state[4];
    mState_constraint_upperbound[epsi_start] = state[5];

    // object that computes objective and constraints
    mFG_eval.initialise(coeffs);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options,
                                          mState,
                                          mState_lowerbound, mState_upperbound,
                                          mState_constraint_lowerbound, mState_constraint_upperbound,
                                          mFG_eval,
                                          solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    //const auto cost = solution.obj_value;
    //std::cout << "Cost " << cost << std::endl;

    // Create MPC planned trajectory x,y coords for visualization
    mComputedTrajectoryX.clear();
    mComputedTrajectoryY.clear();
    for(int step = 1; step < N; ++step) {
        // every x and y after 2nd trajectory step included
        mComputedTrajectoryX.push_back(solution.x[x_start + step]);
        mComputedTrajectoryY.push_back(solution.x[y_start + step]);
    }

    // to bring back the steering angle within [-1, 1]
    mSteerValueResult = - solution.x[delta_start] / utl::deg2rad(25.0);
    mThrottleResult = solution.x[a_start]; // 1st actuation throttle

}

