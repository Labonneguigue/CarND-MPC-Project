#include "MPC.h"
#include "Eigen-3.3/Eigen/Core"
#include "utils.h"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 8;
double dt = 0.15;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set in [mph].
double ref_v = 80;

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {
        // TODO: implement MPC
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.
        // The part of the cost based on the reference state.

        // These weights are tunable to improve the response/trajectory of the car
        const double cteErrorWeight = 1.0;//20.0;
        const double epsiErrorWeight = 1.0;//2000.0;
        const double deltaSpeedErrorWeight = 1.0;//1500.0;
        const double steeringErrorWeight = 1.0;//20000.0;
        const double accelerationErrorWeight = 1.0;//100.0;
        const double diffSteeringErrorWeight = 1.0;//100.0;
        const double diffAccelErrorWeight = 1.0;//500.0;

        for (int t = 0; t < N; t++) {
            fg[0] += cteErrorWeight * CppAD::pow(vars[cte_start + t], 2); // Penalty for cross-track error
            fg[0] += epsiErrorWeight * CppAD::pow(vars[epsi_start + t], 2); // Penalty for angle error
            fg[0] += deltaSpeedErrorWeight * CppAD::pow(vars[v_start + t] - ref_v, 2); // Penalty for speed difference
        }

        // Minimize the use of actuators.
        for (int t = 0; t < N - 1; t++) {
            fg[0] += steeringErrorWeight * CppAD::pow(vars[delta_start + t], 2);  // Penalty for steering the wheels
            fg[0] += accelerationErrorWeight * CppAD::pow(vars[a_start + t], 2);      // Penalty for accelerating or breaking
            fg[0] += 10*CppAD::pow(vars[delta_start + t] * vars[v_start+t], 2);
        }

        // Minimize the value gap between sequential actuations.
        for (int t = 0; t < N - 2; t++) {
            fg[0] += diffSteeringErrorWeight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += diffAccelErrorWeight * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }

        //
        // Setup Constraints
        //
        // NOTE: In this section you'll setup the model constraints.

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints
        for (int t = 1; t < N; t++) {
            // The state at time t+1 .
            const AD<double> x1 = vars[x_start + t];
            const AD<double> y1 = vars[y_start + t];
            const AD<double> psi1 = vars[psi_start + t];
            const AD<double> v1 = vars[v_start + t];
            const AD<double> cte1 = vars[cte_start + t];
            const AD<double> epsi1 = vars[epsi_start + t];

            // The state at time t.
            const AD<double> x0 = vars[x_start + t - 1];
            const AD<double> y0 = vars[y_start + t - 1];
            const AD<double> psi0 = vars[psi_start + t - 1];
            const AD<double> v0 = vars[v_start + t - 1];
            const AD<double> cte0 = vars[cte_start + t - 1];
            const AD<double> epsi0 = vars[epsi_start + t - 1];

            // Only consider the actuation at time t.
            const AD<double> delta0 = vars[delta_start + t - 1];
            const AD<double> a0 = vars[a_start + t - 1];

            const AD<double> f0 = coeffs[0] + (coeffs[1] * x0) + (coeffs[2] * pow(x0, 2)) + (coeffs[3] * pow(x0, 3));
            const AD<double> psides0 = CppAD::atan(coeffs[1] + (2 * coeffs[2] * x0) + (3 * coeffs[3] * pow(x0, 2)));

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // Recall the equations for the model:
            // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
            // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
            // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
            // v_[t+1] = v[t] + a[t] * dt
            // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
            // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
            fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
            fg[1 + cte_start + t] = cte1 - ((y0 - f0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC(double artificialLatency)
    : mStateSize(6U)
    , mNbActuators(2U)
    , mArtificialLatencyMs(artificialLatency)
    , mRuntime(mArtificialLatencyMs)
    , N(8)
    , dt(0.15)
    , actuatorsArraySize(static_cast<size_t>( mNbActuators * ( N - 1 ) ))
    , constraintsArraySize(static_cast<size_t>( N * mStateSize ))
    , nbVars(actuatorsArraySize + constraintsArraySize)
    , mState(nbVars)
{}

MPC::~MPC()
    {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;

    typedef CPPAD_TESTVECTOR(double) Dvector;

    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9

    // Number of actuators * actuation steps
    //const size_t actuatorsArraySize = static_cast<size_t>( mNbActuators * ( N - 1 ) );
    // TODO: Set the number of constraints
    //const size_t constraintsArraySize = static_cast<size_t>( N * mStateSize );

    //const size_t nbVars = actuatorsArraySize + constraintsArraySize;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    //Dvector vars(nbVars);
    for (int i = 0; i < nbVars; i++) {
        mState[i] = 0.0;
    }

    // I set the initial state
    mState[x_start] = state[0];
    mState[y_start] = state[1];
    mState[psi_start] = state[2];
    mState[v_start] = state[3];
    mState[cte_start] = state[4];
    mState[epsi_start] = state[5];


    Dvector vars_lowerbound(nbVars);
    Dvector vars_upperbound(nbVars);
    // TODO: Set lower and upper limits for variables.

    // No limit for x, y, psi, v, cte and epsi
    for (int i = 0; i < delta_start; i++) {
        /*vars_lowerbound[i] = numeric_limits<double>::min();
        vars_upperbound[i] = numeric_limits<double>::max();*/
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // Limit steering delta to stay within [-25 deg, +25 deg] range
    for (int i = delta_start; i < a_start; i++) {
        /*vars_lowerbound[i] = utl::deg2rad(-25.0);
        vars_upperbound[i] = utl::deg2rad(25.0);*/
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Limit to the acceleration to be the maximum possible allowed by the car
    for (int i = a_start; i < nbVars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(constraintsArraySize);
    Dvector constraints_upperbound(constraintsArraySize);
    for (int i = 0; i < constraintsArraySize; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    // Set the initial state lower limits to current state vector values
    constraints_lowerbound[x_start] = state[0];
    constraints_lowerbound[y_start] = state[1];
    constraints_lowerbound[psi_start] = state[2];
    constraints_lowerbound[v_start] = state[3];
    constraints_lowerbound[cte_start] = state[4];
    constraints_lowerbound[epsi_start] = state[5];
    // Set the initial state upper limits to current state vector values
    constraints_upperbound[x_start] = state[0];
    constraints_upperbound[y_start] = state[1];
    constraints_upperbound[psi_start] = state[2];
    constraints_upperbound[v_start] = state[3];
    constraints_upperbound[cte_start] = state[4];
    constraints_upperbound[epsi_start] = state[5];

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
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
                                          vars_lowerbound, vars_upperbound,
                                          constraints_lowerbound, constraints_upperbound,
                                          fg_eval,
                                          solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    const auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    // TODO: Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.

    // Create MPC planned trajectory x,y coords for visualization
    mComputedTrajectoryX.clear();
    mComputedTrajectoryY.clear();
    for(int step = 1; step < N; ++step) {
        // every x and y after 2nd trajectory step included
        mComputedTrajectoryX.push_back(solution.x[x_start + step]);
        mComputedTrajectoryY.push_back(solution.x[y_start + step]);
    }

    // Return vector of the 1st step actuations for steering and throttle commands
    std::vector<double> mpcActuation(2);

    // to bring back the steering angle within [-1, 1]
    mpcActuation[0] = solution.x[delta_start] / utl::deg2rad(25.0);
    mpcActuation[1] = solution.x[a_start]; // 1st actuation throttle
    
    return mpcActuation;
}

