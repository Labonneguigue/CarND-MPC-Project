#ifndef MPC_H
#define MPC_H

#include <vector>
#include "FG_eval.h"

using namespace std;
using CppAD::AD;

class MPC {
public:

    typedef CPPAD_TESTVECTOR(double) Dvector;

    /** Default constructor
     *
     */
    MPC(double artificialLatency = 100.0);

    /** Default destructor
     *
     */
    virtual ~MPC();

    /** Solve the model given an initial state and polynomial coefficients.
     *
     * @note Throttle and steering values are returned by public class variables
     */
    void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

    /** Returns the x axis of the trajectory that optimises the constraints
     *
     * @return mComputedTrajectoryX  vector of x coordinates
     */
    inline std::vector<double>& computedTrajectoryX(){
        return mComputedTrajectoryX;
    }

    /** Returns the y axis of the trajectory that optimises the constraints
     *
     * @return mComputedTrajectoryY  vector of y coordinates
     */
    inline std::vector<double>& computedTrajectoryY(){
        return mComputedTrajectoryY;
    }

    /** Returns the size of the state
     *
     */
    inline unsigned int stateSize(){
        return mStateSize;
    }

    /** Returns the arbitrary artificial latency that is
     *  supposed to represent the actuators latency.
     *
     * @return artificalLatencyMs  Expressed in milliseconds.
     */
    inline double latency(){
        return mArtificialLatencyMs;
    }

    /** Trivial setter of the overall runtime of the Model Predictive Controller
     *
     */
    inline void runtime(double runtime){
        mRuntime = runtime;
    }

    /** Simple getter to obtain the runtime of the overall execution
     *  with the incurred artificial latency
     */
    inline double runtime(){
        return mRuntime;
    }

public:

    double dt; ///< Timesteps duration

    // This value assumes the model presented in the classroom is used.
    //
    // It was obtained by measuring the radius formed by running the vehicle in the
    // simulator around in a circle with a constant steering angle and velocity on a
    // flat terrain.
    //
    // Lf was tuned until the the radius formed by the simulating the model
    // presented in the classroom matched the previous radius.
    // Length from front of the car to CoG that has a similar radius.
    double Lf;

    double mSteerValueResult; ///< Result steer_value of the solve method to be applied to the car
    double mThrottleResult; ///< Result throttle value of the solve method to be applied to the car

private:

    unsigned int mStateSize; ///< Number of parameters given to the solver.
    unsigned int mNbActuators; ///< Number of actuator values given to the simulator.

    std::vector<double> mComputedTrajectoryX; ///< x coordinate of the MPC trajectory.
    std::vector<double> mComputedTrajectoryY; ///< y coordinate of the MPC trajectory.

    const double mArtificialLatencyMs; ///< latency of the actuators expressed in [ms].

    double mRuntime; ///< runtime of the overall callback mechanism in [ms].

    size_t N; ///< Number of steps to predict ahead

    double mTargetSpeed; ///< Top cruise speed to match

    const size_t actuatorsArraySize; ///< Size of the actuator array
    const size_t constraintsArraySize; ///< Size of the array of the constraints
    const size_t nbVars; ///< Number of model variables

    Dvector mState; ///< Vector keeping track of all the state variables

    Dvector mState_lowerbound; ///< Vector of State variables lower limits
    Dvector mState_upperbound; ///< Vector of State variables upper limits
    Dvector mState_constraint_lowerbound; ///< Vector of constraints lower limits
    Dvector mState_constraint_upperbound; ///< Vector of constraints upper limits

    FG_eval mFG_eval; ///< Solver

};

#endif /* MPC_H */
