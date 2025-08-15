#pragma once
#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fstream>
#include <atomic>

class Logger {
public:
    Logger();
    ~Logger();

    void push(const std::string& message);

private:
    void run();

    std::queue<std::string> _messages;
    std::mutex _mutex;
    std::condition_variable _cv;
    std::ofstream _file;
    std::atomic<bool> _running{true};
    std::thread _thread;
};
