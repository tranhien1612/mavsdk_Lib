#include "Drone.h"
#include <iostream>
#include <thread>

int main() {
    try {
        
        Drone drone("udpin://0.0.0.0:14550");
        drone.check_health();

        drone.takeoff(10.0);

        drone.start_Offboard_Mode();
        std::this_thread::sleep_for(std::chrono::seconds(10));
        drone.stop_Offboard_Mode();
        std::this_thread::sleep_for(std::chrono::seconds(10));

        drone.land();

        // drone.rtl();


        while(1){
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
