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
    MPC();

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

    inline int stateSize(){
        return mStateSize;
    }

private:

    int mStateSize;
    int mNbActuators;

    std::vector<double> mComputedTrajectoryX;
    std::vector<double> mComputedTrajectoryY;
    
};

#endif /* MPC_H */
