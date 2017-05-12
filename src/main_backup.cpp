#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <random>
#include <stdlib.h>


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
    return M_PI;
}
double deg2rad(double x)
{
    return x * pi() / 180;
}
double rad2deg(double x)
{
    return x * 180 / pi();
}

double cte_old = 0.0;
double d_cte = 0.0;
double cte_int = 0.0;
double steer_value = 0.0;
double coeff_p = -0.035;
double coeff_i = -0.0015;
double coeff_d = -0.3;
int kk = 0;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos)
    {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos)
    {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main()
{
    uWS::Hub h;

    PID pid;
    // TODO: Initialize the pid variable.

    h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "")
            {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry")
                {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    double angle = std::stod(j[1]["steering_angle"].get<std::string>());

                    kk += 1;
                    d_cte = cte - cte_old;
                    cte_int += cte;
                    steer_value = coeff_p*cte + coeff_d*d_cte + coeff_i*cte_int;

                    if (kk == 50)
                    {
                        std::cout << "Done: kp= " << coeff_p << " ki= " << coeff_i << " kd= " << coeff_d << " cte= " << cte << std::endl;
                        kk = 0;
                        cte_old = 0.0;
                        d_cte = 0.0;
                        cte_int = 0.0;
                        steer_value = 0.0;
                        cte = 0.0;
                        std::default_random_engine gen;
                        std::normal_distribution<double> dist_p(coeff_p,0.01);
                        std::normal_distribution<double> dist_i(coeff_i,0.001);
                        std::normal_distribution<double> dist_d(coeff_d,0.1);
                        coeff_p = dist_p(gen);
                        coeff_i = dist_i(gen);
                        coeff_d = dist_d(gen);

                        std::string reset_msg = "42[\"reset\",{}]";
                        ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                        sleep(5);

                    }


                    /*
                    * TODO: Calcuate steering value here, remember the steering value is
                    * [-1, 1].
                    * NOTE: Feel free to play around with the throttle and speed. Maybe use
                    * another PID controller to control the speed!
                    */

                    // DEBUG
                    std::cout << "kk: " << kk << " CTE: " << cte << " dCTE: " << d_cte << " intCTE: " << cte_int <<" Steering Value: " << steer_value << std::endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    if (fabs(cte) > 2.0)
                    {
                        msgJson["throttle"] = 0.1;
                    }
                    else
                    {
                        msgJson["throttle"] = 0.2;
                    }
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    //std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    cte_old = cte;

                }
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
    {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}

void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws)
{
    // reset
    std::string msg("42[\"reset\", {}]");
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}
