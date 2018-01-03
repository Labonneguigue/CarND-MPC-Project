
#include "FG_eval.h"

// TODO: Set the timestep length and duration
size_t N = 8;
//double dt = 0.15;

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
//const double Lf = 2.67;

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


FG_eval::FG_eval(size_t N,
                 double dt,
                 double Lf,
                 double targetSpeed)
    : N(N)
    , dt(dt)
    , Lf(Lf)
    , mTargetSpeed(targetSpeed)
    , cteErrorWeight(20.0)//20.0;
    , epsiErrorWeight(20.0)//2000.0;
    , deltaSpeedErrorWeight(1.0)//1500.0;
    , steeringErrorWeight(1.0)//20000.0;
    , accelerationErrorWeight(1.0)//100.0;
    , diffSteeringErrorWeight(1.0)//100.0;
    , diffAccelErrorWeight(1.0)//500.0;
    , steerAccelErrorWeight(100.0)
{}

void FG_eval::initialise(Eigen::VectorXd coeffs)
{
    this->coeffs = coeffs;
}

void FG_eval::operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    // The part of the cost based on the reference state.

    for (int t = 0; t < N; t++) {
        fg[0] += cteErrorWeight * CppAD::pow(vars[cte_start + t], 2); // Penalty for cross-track error
        fg[0] += epsiErrorWeight * CppAD::pow(vars[epsi_start + t], 2); // Penalty for angle error
        fg[0] += deltaSpeedErrorWeight * CppAD::pow(vars[v_start + t] - mTargetSpeed, 2); // Penalty for speed difference
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
        fg[0] += steeringErrorWeight * CppAD::pow(vars[delta_start + t], 2); // Penalty for steering the wheels
        fg[0] += accelerationErrorWeight * CppAD::pow(vars[a_start + t], 2); // Penalty for accelerating or breaking
        fg[0] += steerAccelErrorWeight*CppAD::pow(vars[delta_start + t] * vars[v_start+t], 2);  // Penalty for steering and accelerating at the same time
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
