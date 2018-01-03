#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "utils.h"

#define DEBUG 0

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {


        std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

        // Artificial latency - Represents the actuators latency in [ms]
        const double artificialtLatencyMs = 100.0; // TODO: use the one in MPC.cpp (duplicate)

        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    vector<double> ptsX = j[1]["ptsx"];
                    vector<double> ptsY = j[1]["ptsy"];
                    double px = j[1]["x"];    // car global x coordinate [meter]
                    double py = j[1]["y"];    // car global y coordinate [meter]
                    double psi = j[1]["psi"]; // car global heading angle [rad]
                    double v = static_cast<double>(j[1]["speed"]); // car speed in [mph]
                    double steering_angle = j[1]["steering_angle"];
                    double throttle = j[1]["throttle"];

#if DEBUG
                    std::cout << fixed << setprecision(4);
                    std::cout << "***************************************\n";
                    std::cout << setw(6) << "x :" << setw(6) << px << "\t y : " << setw(6) << py << "\n";
                    std::cout << setw(6) << "psi :" << setw(6) << psi << "\t v : " << setw(6) << v << "\n";
                    std::cout << "----------------\n";
#endif
                    //*************************************************************
                    // TODO: display steering angle and throttle and check predicted
                    // values extrapolated due to the latency
                    //*************************************************************

                    // I extrapolate the car's position after the estimated latency
                    // Latency is 100ms.
                    // Here are the motion's equations:
                    // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
                    // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
                    // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
                    // v_[t+1] = v[t] + a[t] * dt
                    // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
                    // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
                    const double latencyS = mpc.runtime() / 1000.0;
                    const double Lf = 2.67; // TODO : remove from here (duplicate)
                    const double predV = v + throttle * latencyS;
                    const double predPsi = psi + (predV / Lf) * (-steering_angle) * latencyS;
                    const double predX = px + (predV * cos(predPsi) * latencyS);
                    const double predY = py + (predV * sin(predPsi) * latencyS);

                    #if DEBUG
                    std::cout << setw(10) << "predX :" << setw(10) << predX
                    << setw(10) << "\t predY : " << setw(10) << predY << "\n";
                    std::cout << setw(10) << "predPsi : " << setw(10) << predPsi
                    << setw(10) << "\t predV :" << setw(6) << predV << "\n\n";
                    #endif

                    // Construction of eigen vectors that contain the waypoints
                    // The waypoints constitute the optimal trajectory in the
                    // vehicle coordinates.
                    Eigen::VectorXd ptsXVehicle(ptsX.size());
                    Eigen::VectorXd ptsYVehicle(ptsY.size());
                    assert(ptsXVehicle.size() == ptsYVehicle.size());
                    // I preprocess the waypoints to remove the current position of the car
                    // This step helps for the polynomial fitting
                    // Deduce car's position and then rotate every points by the angle of the car
#if 0
                    Eigen::Matrix2d rotation;
                    rotation << cos(predPsi), sin(predPsi),
                               -sin(predPsi), cos(predPsi);
                    Eigen::Vector2d waypoint;
                    for (int i = 0; i < ptsXVehicle.size(); i++) {
//                        double x = ptsX[i] - predX;
//                        double y = ptsY[i] - predY;
//                        ptsXVehicle[i] = ( x * cos(predPsi) ) - (y * sin(predPsi));
//                        ptsYVehicle[i] = ( x * sin(predPsi) ) + (y * cos(predPsi));
                        waypoint = rotation * Eigen::Vector2d(ptsX[i] - predX,
                                                              ptsY[i] - predY);
                        ptsXVehicle[i] = waypoint[0];
                        ptsYVehicle[i] = waypoint[1];
                    }
#else
                    Eigen::Matrix2d rotation;
                    rotation << cos(-psi), -sin(-psi),
                                sin(-psi), cos(-psi);
                    Eigen::Vector2d waypoint;
                    for (int i = 0; i < ptsXVehicle.size(); i++) {
                        //                        double x = ptsX[i] - predX;
                        //                        double y = ptsY[i] - predY;
                        //                        ptsXVehicle[i] = ( x * cos(predPsi) ) - (y * sin(predPsi));
                        //                        ptsYVehicle[i] = ( x * sin(predPsi) ) + (y * cos(predPsi));
                        waypoint = rotation * Eigen::Vector2d(ptsX[i] - px,
                                                              ptsY[i] - py);
                        ptsXVehicle[i] = waypoint[0];
                        ptsYVehicle[i] = waypoint[1];
                    }
#endif

                    #if 0
                    std::cout << "ptsX : ";
                    for (int pts = 0 ; pts < ptsX.size() ; pts++ ){
                        std::cout << ptsX[pts] << "  ";
                    }
                    std::cout << "\n";
                    std::cout << "ptsY : ";
                    for (int pts = 0 ; pts < ptsY.size() ; pts++ ){
                        std::cout << ptsY[pts] << "  ";
                    }
                    std::cout << "\n";
                    std::cout << "ptsXVehicle : ";
                    for (int pts = 0 ; pts < ptsXVehicle.size() ; pts++ ){
                        std::cout << ptsXVehicle[pts] << "  ";
                    }
                    std::cout << "\n";
                    std::cout << "ptsYVehicle : ";
                    for (int pts = 0 ; pts < ptsYVehicle.size() ; pts++ ){
                        std::cout << ptsYVehicle[pts] << "  ";
                    }
                    std::cout << "\n";
                    #endif

                    // I now fit a 2nd order polynomial on the waypoints/trajectory
                    auto coeffs = polyfit(ptsXVehicle, ptsYVehicle, 3);
                    double cte = coeffs[0]; //polyeval(coeffs, 0);      // Cross-track error
                    double epsi = -atan(coeffs[1]);         // Angle error (psi)

                    for (int coeff = 0 ; coeff < coeffs.size() ; coeff++ ){
                        assert(!isnan(coeffs[coeff]));
                    }

                    #if DEBUG
                    std::cout << "coeffs : ";
                    for (int coeff = 0 ; coeff < coeffs.size() ; coeff++ ){
                        std::cout << coeffs[coeff] << "  ";
                    }
                    std::cout << "\n";

                    std::cout << "cte : " << cte << "\n";
                    std::cout << "epsi : " << epsi << "\n\n";
                    #endif

                    const double dt = 0.1;
                    const double current_px = 0.0 + v * dt;
                    const double current_py = 0.0;
                    const double current_psi = 0.0 + v * (-steering_angle) / Lf * dt;
                    const double current_v = v + throttle * dt;
                    const double current_cte = cte + v * sin(epsi) * dt;
                    const double current_epsi = epsi + v * (-steering_angle) / Lf * dt;

                    // I set up the state vector
                    Eigen::VectorXd state(mpc.stateSize());
                    //state << 0, 0, 0, predV, cte, epsi;
                    state << current_px, current_py, current_psi, current_v, current_cte, current_epsi;

                    // I call the MPC Solver to compute the optimal trajectory
                    auto result = mpc.Solve(state, coeffs);
                    double steer_value = -result[0];
                    double throttle_value = result[1];

                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;

                    #if DEBUG
                    std::cout << "steering angle : " << steer_value << "  throttle : " << throttle_value << "\n";
                    #endif

                    // Display the MPC predicted trajectory.
                    // Points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    msgJson["mpc_x"] = mpc.computedTrajectoryX();
                    msgJson["mpc_y"] = mpc.computedTrajectoryY();


                    // Display the waypoints/reference line
                    // Points are in reference to the vehicle's coordinate system.
                    // The points in the simulator are connected by a Yellow line.
                    vector<double> next_x_vals(ptsXVehicle.data(),
                                               ptsXVehicle.data() + ptsXVehicle.size());
                    vector<double> next_y_vals(ptsYVehicle.data(),
                                               ptsYVehicle.data() + ptsYVehicle.size());
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";

                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car doesn't actuate the commands instantly.
                    this_thread::sleep_for(std::chrono::duration<double, std::milli>(artificialtLatencyMs));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);


                    // Effective latency = program time + added latency
                    double runtime = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count());
                    mpc.runtime(runtime);

                    #if DEBUG
                    std::cout << "Effective latency [ms]: " << runtime << "\n";
                    std::cout << "***************************************\n\n";
                    #endif

                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
    
    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
