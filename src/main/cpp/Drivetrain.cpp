#include "drivetrain.hpp"
#include <thread>
// #include <cmath>
Drivetrain::Drivetrain()
{
    navx->ZeroYaw();
    navx->GetYaw();
    
}

// get robot orientation from rio / navX
double Drivetrain::get_angle()
{
    return ngr::deg2rad(navx->GetYaw());
}
// drive in specified direction
void Drivetrain::drive(Twist_I const& twist)
{
    Twist_R const twist_r { twist, get_angle() };
    // std::cout  << "Yaw: " << twist_r.dtheta << '\n';
    // std::vector<std::thread> t;
    for(auto&& wheel : wheels)
        wheel->drive(twist_r);  //t.emplace_back([&] { wheel->drive(twist_r); }); We believe this removes multithreading... 
    // for(auto& ts : t)
    // {
    // ts.join();
    // }

}

void Drivetrain::print()
{
    int i = 0;
    for(auto& wheel : wheels)

    // std::cout << "Wheel " << ++i << ": " << ngr::rad2deg(wheel->get_angle() + wheel->alpha + wheel->beta) << '\n';
    // std::cout << "Twist R is: " << Wheel::get_vector_for() << '\n';
}