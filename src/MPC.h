#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
public:

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
     * @return Returns the first actuations.
     */
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

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

    inline void runtime(double runtime){
        mRuntime = runtime;
    }

    inline double runtime(){
        return mRuntime;
    }

private:

    unsigned int mStateSize; ///< Number of parameters given to the solver.
    unsigned int mNbActuators; ///< Number of actuator values given to the simulator.

    std::vector<double> mComputedTrajectoryX; ///< x coordinate of the MPC trajectory.
    std::vector<double> mComputedTrajectoryY; ///< y coordinate of the MPC trajectory.

    const double mArtificialLatencyMs; ///< latency of the actuators expressed in [ms].

    double mRuntime; ///< runtime of the overall callback mechanism in [ms].

};

#endif /* MPC_H */
