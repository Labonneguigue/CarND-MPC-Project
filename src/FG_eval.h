#ifndef FG_EVAL_HEADER
#define FG_EVAL_HEADER

#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "utils.h"
#include "config.h"

using CppAD::AD;


class FG_eval {
public:

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    FG_eval(size_t N,
            double dt,
            double Lf,
            double targetSpeed);

    void initialise(Eigen::VectorXd coeffs);

    void operator()(ADvector& fg, const ADvector& vars);

private:

    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    size_t N;
    double dt;
    double Lf;
    double mTargetSpeed;

    // These weights are tunable to improve the response/trajectory of the car
    const double cteErrorWeight;
    const double epsiErrorWeight;
    const double deltaSpeedErrorWeight;
    const double steeringErrorWeight;
    const double accelerationErrorWeight;
    const double diffSteeringErrorWeight;
    const double diffAccelErrorWeight;
    const double steerAccelErrorWeight;

};

#endif //FG_EVAL_HEADER
