#include "Logger.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <iostream>
#include <string>

Logger::Logger() {
    auto t = std::time(nullptr);
    char filename[64];
    std::strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S.log", std::localtime(&t));

    _file.open(filename, std::ios::out | std::ios::app);
    _thread = std::thread(&Logger::run, this);
}

Logger::~Logger() {
    _running = false;
    _cv.notify_all();
    if (_thread.joinable()) {
        _thread.join();
    }
    if (_file.is_open()) {
        _file.close();
    }
}

void Logger::push(const std::string& message) {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << "[" << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S") << "] " << message;

    {
        std::lock_guard<std::mutex> lock(_mutex);
        _messages.push(ss.str());
    }
    _cv.notify_one();
}

void Logger::run() {
    while (_running || !_messages.empty()) {
        std::unique_lock<std::mutex> lock(_mutex);
        _cv.wait(lock, [this] { return !_messages.empty() || !_running; });

        while (!_messages.empty()) {
            _file << _messages.front() << std::endl;
            _messages.pop();
        }
    }
}
