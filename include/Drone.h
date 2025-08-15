#pragma once
#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <fstream>
#include <memory>

#include <mavsdk/mavsdk.h>
#include <mavsdk/log_callback.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/geofence/geofence.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action_server/action_server.h>
#include <mavsdk/plugins/mavlink_direct/mavlink_direct.h>
#include <mavsdk/plugins/server_utility/server_utility.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "Logger.h"

class Drone {
public:
    Drone(const std::string& connection_url);
    
    std::string flight_mode_to_string(mavsdk::Telemetry::FlightMode mode);
    void subcribe_callback();
    void check_health();
    void takeoff(float altitude);
    void land();
    void rtl();

    void start_Offboard_Mode();
    void stop_Offboard_Mode();

    //Mission
    void start_mission();
    void pause_mission();

    std::vector<mavsdk::Mission::MissionItem> create_mission();
    void upload_mission(std::vector<mavsdk::Mission::MissionItem> &mission_items);
   
    void download_mission();


private:
    mavsdk::Mavsdk _mavsdk;
    std::shared_ptr<mavsdk::System> _system;
    std::shared_ptr<mavsdk::Action> _action;
    std::shared_ptr<mavsdk::Telemetry> _telemetry;
    std::shared_ptr<mavsdk::MavlinkDirect> _mavlink_direct;  
    std::shared_ptr<mavsdk::Offboard> _offboard;  
    std::shared_ptr<mavsdk::Mission> _mission;
    std::vector<std::shared_ptr<mavsdk::Mission::MissionItem>> _mission_items;

    mavsdk::Telemetry::FlightMode _old_flight_mode = mavsdk::Telemetry::FlightMode::Unknown;
    mavsdk::Telemetry::FlightMode _current_flight_mode;

    mavsdk::Telemetry::Position current_pos;

    std::shared_ptr<Logger> _logger;
    void log(const std::string& message);
};
