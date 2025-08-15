#include "Drone.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <stdexcept>

int extract_value(const std::string& json_str, const std::string& key) {
    auto pos = json_str.find("\"" + key + "\":");
    if (pos == std::string::npos) return -1;
    pos += key.size() + 3; // skip over "key":
    size_t end_pos = json_str.find_first_of(",}", pos);
    return std::stoi(json_str.substr(pos, end_pos - pos));
}

std::string mode2str(int mode){
    std::string str;
    switch(mode){
        case 0: 
            str = "STABILIZE"; break;
        case 1: 
            str = "ACRO"; break;
        case 2: 
            str = "ALT_HOLD"; break;
        case 3: 
            str = "AUTO"; break;
        case 4: 
            str = "GUIDED"; break;
        case 5: 
            str = "LOITER"; break;
        case 6: 
            str = "RTL"; break; 
        case 7: 
            str = "CIRCLE"; break;
        case 8: 
            str = "POSITION"; break;
        case 9: 
            str = "LAND"; break;
        case 10: 
            str = "OF_LOITER"; break;
        case 11: 
            str = "DRIFT"; break;
        case 13: 
            str = "SPORT"; break;
        case 14: 
            str = "FLIP"; break;
        case 15: 
            str = "AUTOTUNE"; break;
        case 16: 
            str = "POSHOLD"; break;
        case 17: 
            str = "BRAKE"; break;
        case 18: 
            str = "THROW"; break;
        case 19: 
            str = "AVOID_ADSB"; break;
        case 20: 
            str = "GUIDED_NOGPS"; break;
        case 21: 
            str = "SMART_RTL"; break;
        case 22: 
            str = "FLOWHOLD"; break;
        case 23: 
            str = "FOLLOW"; break;
        case 24: 
            str = "ZIGZAG"; break;
        case 25: 
            str = "SYSTEMID"; break;
        case 26: 
            str = "AUTOROTATE"; break;
        case 27: 
            str = "AUTO_RTL"; break;
        default:
            str = "Unknown";
    }
    return str;
}


Drone::Drone(const std::string& connection_url)
    : _mavsdk(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation})
{
    _logger = std::make_shared<Logger>();
    auto connection_result = _mavsdk.add_any_connection(connection_url);
    if (connection_result != mavsdk::ConnectionResult::Success) {
        log("Connection failed: " + connection_url);
        throw std::runtime_error("Connection failed: " + std::to_string(static_cast<int>(connection_result)));
    }

    // Wait for system discovery
    while (_mavsdk.systems().empty()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    _system = _mavsdk.systems().at(0);
    if (!_system->is_connected()) {
        this->log("System not connected.");
        return;
    }
    _action = std::make_shared<mavsdk::Action>(*_system);
    _telemetry = std::make_shared<mavsdk::Telemetry>(*_system);
    _mavlink_direct = std::make_shared<mavsdk::MavlinkDirect>(_system);
    _mission = std::make_shared<mavsdk::Mission>(_system);
    _offboard = std::make_shared<mavsdk::Offboard>(_system);

    this->log("Connecting to MAVLink: " + connection_url);
    subcribe_callback();
}

std::string Drone::flight_mode_to_string(mavsdk::Telemetry::FlightMode mode) {
    switch (mode) {
        case mavsdk::Telemetry::FlightMode::Ready: return "Ready";
        case mavsdk::Telemetry::FlightMode::Takeoff: return "Takeoff";
        case mavsdk::Telemetry::FlightMode::Hold: return "Hold";
        case mavsdk::Telemetry::FlightMode::Mission: return "Mission";
        case mavsdk::Telemetry::FlightMode::ReturnToLaunch: return "ReturnToLaunch";
        case mavsdk::Telemetry::FlightMode::Land: return "Land";
        case mavsdk::Telemetry::FlightMode::Offboard: return "Offboard";
        default: return "Unknown";
    }
}

void Drone::subcribe_callback() {

    // _mavlink_direct->subscribe_message("HEARTBEAT", [this](const mavsdk::MavlinkDirect::MavlinkMessage& mav_msg) {
    //         int base_mode = extract_value(mav_msg.fields_json, "base_mode");
    //         int custom_mode = extract_value(mav_msg.fields_json, "custom_mode");
    //         this->log(
    //             "MessageName: " + mav_msg.message_name +
    //             ", system_id: " + std::to_string(mav_msg.system_id) +
    //             ", component_id: " + std::to_string(mav_msg.component_id) +
    //             ", target_component: " + std::to_string(mav_msg.target_component) +
    //             ", base_mode: " + mode2str(base_mode) +
    //             ", custom_mode: " + mode2str(custom_mode)
    //         );
    //     }
    // );

    // _mavlink_direct->subscribe_message("GPS_RAW_INT", [this](const mavsdk::MavlinkDirect::MavlinkMessage& mav_msg) {
    //         double lat = (double)extract_value(mav_msg.fields_json, "lat") / 1e7;
    //         double lon = (double)extract_value(mav_msg.fields_json, "lon") / 1e7;
    //         double alt = (double)extract_value(mav_msg.fields_json, "alt") / 1000.0 ;
    //         this->log(
    //             "MessageName: " + mav_msg.message_name +
    //             ", Lat: " + std::to_string(lat) +
    //             ", Lon: " + std::to_string(lon) +
    //             ", Alt: " + std::to_string(alt)
    //         );
    //     }
    // );

    _telemetry->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode mode) {
        _current_flight_mode = mode;
        if (_old_flight_mode != mode) {
            this->log("Flight Mode Change from " + flight_mode_to_string(_old_flight_mode) + " to " + flight_mode_to_string(_current_flight_mode));
            _old_flight_mode = mode;
        }
    });

    static auto last_log_time = std::chrono::steady_clock::now();
    _telemetry->subscribe_position([this](mavsdk::Telemetry::Position pos) {
        this->current_pos = pos;
        auto now = std::chrono::steady_clock::now();
        if (now - last_log_time >= std::chrono::seconds(2)) { //Save position every 2 seconds
            this->log("Drone Position: Lat: " + std::to_string(pos.latitude_deg) +
                    ", Lon: " + std::to_string(pos.longitude_deg) +
                    ", Alt: " + std::to_string(pos.relative_altitude_m) + "m");
            last_log_time = now;
        }
    });
}

void Drone::check_health() {
    using namespace std::chrono_literals;
    this->log("Vehicle Health cheking...");

    mavsdk::Telemetry::Health check_health = _telemetry->health();
    if (!check_health.is_gyrometer_calibration_ok)      this->log("  - Gyro requires calibration.");
    if (!check_health.is_accelerometer_calibration_ok)  this->log("  - Accelerometer requires calibration.");
    if (!check_health.is_magnetometer_calibration_ok)   this->log("  - Magnetometer (compass) requires calibration.");

    while (!_telemetry->health_all_ok()) {
        auto health = _telemetry->health();
        this->log("Vehicle not ready");
        if (!health.is_global_position_ok)  this->log("  - Waiting for GPS fix");
        if (!health.is_local_position_ok)   this->log("  - Waiting for local position");
        if (!health.is_home_position_ok)    this->log("  - Waiting for home position");
        std::this_thread::sleep_for(1s);
    }
}

void Drone::takeoff(float altitude) {
    this->log("Arming ...");
    if (_action->arm() != mavsdk::Action::Result::Success) {
        this->log("Arming failed!");
        return;
    }

    _action->set_takeoff_altitude(altitude);

    this->log("Takeoff...");
    if (_action->takeoff() != mavsdk::Action::Result::Success) {
        this->log("Takeoff failed!");
        return;
    }

    this->log("Wait Takeoff until altitude = " + std::to_string(altitude));
    while (this->current_pos.relative_altitude_m <= altitude){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    this->log("Takeoff successful.");
}

void Drone::land() {
    this->log("Landing...");
    if (_action->land() != mavsdk::Action::Result::Success) {
        this->log("Landing failed!");
    }
    while (_telemetry->armed()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    this->log("Disarmed.");
}

void Drone::rtl() {
    this->log("Return to Launch...");
    if (_action->return_to_launch() != mavsdk::Action::Result::Success) {
        this->log("RTL failed!");
    }
}

/* ------------------------- offboard ------------------------- */
void Drone::start_Offboard_Mode(){
    this->log("Start Offboard Mode");
    _offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});

    mavsdk::Offboard::Result result = _offboard->start();
    if (result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard::start() failed: " << result << '\n';
    }

    // _offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 270.0f}); //Turn to face West
    _offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 60.0f}); //Turn clock-wise at 60 degrees per second
    // _offboard->set_velocity_body({5.0f, 0.0f, 0.0f, 0.0f});
    // _offboard->set_velocity_body({5.0f, 0.0f, 0.0f, 30.0f}); //fly circle
    // _offboard->set_velocity_body({0.0f, -5.0f, 0.0f, -30.0f}); //// Fly a circle sideways
}

void Drone::stop_Offboard_Mode(){
    mavsdk::Offboard::Result result = _offboard->stop();
    if (result != mavsdk::Offboard::Result::Success) {
        std::cerr << "Offboard::stop() failed: " << result << '\n';
    }
    this->log("Stop Offboard Mode");
}

/* ------------------------- Mission ------------------------- */
mavsdk::Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    mavsdk::Mission::MissionItem::CameraAction camera_action)
{
    mavsdk::Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    new_item.is_fly_through = is_fly_through;
    new_item.gimbal_pitch_deg = gimbal_pitch_deg;
    new_item.gimbal_yaw_deg = gimbal_yaw_deg;
    new_item.camera_action = camera_action;
    return new_item;
}

void Drone::start_mission(){

    _mission->subscribe_mission_progress([](mavsdk::Mission::MissionProgress mission_progress) {
        std::cout << "Mission status update: " << mission_progress.current << " / "
                  << mission_progress.total << '\n';
    });

    const mavsdk::Mission::Result result = _mission->start_mission();
    if (result != mavsdk::Mission::Result::Success) {
        std::cout << "Mission start failed (" << result << "), exiting." << '\n';
        return ;
    }
    std::cout << "Started mission." << '\n';
}

void Drone::pause_mission(){
    std::cout << "Pausing mission..." << '\n';
    const mavsdk::Mission::Result result = _mission->pause_mission();
    if (result != mavsdk::Mission::Result::Success) {
        std::cout << "Failed to pause mission (" << result << ")" << '\n';
    } else {
        std::cout << "Mission paused." << '\n';
    }
}

std::vector<mavsdk::Mission::MissionItem> Drone::create_mission(){
    std::vector<mavsdk::Mission::MissionItem> mission_items;
    mission_items.clear();
    mission_items.push_back(make_mission_item(
        22.68170327054473,
        120.2456490218639658,
        10.0f,
        5.0f,
        false,
        0.0f,
        0.0f,
        mavsdk::Mission::MissionItem::CameraAction::None));

    return mission_items;
}

void Drone::upload_mission(std::vector<mavsdk::Mission::MissionItem> &mission_items) {
    // Upload mission
    std::cout << "Uploading mission..." << '\n';
    mavsdk::Mission::MissionPlan mission_plan;
    mission_plan.mission_items = mission_items;
    const mavsdk::Mission::Result result = _mission->upload_mission(mission_plan);
    if (result != mavsdk::Mission::Result::Success) {
        std::cout << "Mission upload failed (" << result << "), exiting." << '\n';
        return;
    }

    std::cout << "Mission uploaded." << '\n';

}

void Drone::download_mission(){
    std::cout << "Downloading mission." << '\n';
    std::pair<mavsdk::Mission::Result, mavsdk::Mission::MissionPlan> result = _mission->download_mission();

    if (result.first != mavsdk::Mission::Result::Success) {
        std::cout << "Mission download failed (" << result.first << "), exiting." << '\n';
        return;
    }
    std::cout << "Mission downloaded (MissionItems: "<< result.second.mission_items.size() << ")" << '\n';

}




/* ------------------------- logging ------------------------- */
void Drone::log(const std::string& message) {
    if (_logger) {
        _logger->push(message);
    }
    std::cout << message << std::endl; // Also print to stdout
}