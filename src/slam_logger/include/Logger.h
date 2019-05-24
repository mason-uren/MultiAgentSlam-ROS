#include <utility>

//
// Created by Mason U'Ren on 2019-04-25.
//

#ifndef MULTIAGENTSLAM_LOGGER_H
#define MULTIAGENTSLAM_LOGGER_H

#include <iostream>
#include <array>
#include <fstream>
#include <mutex>
#include <sstream>
#include <fcntl.h>
#include <time.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

#include <include/SharedMemoryStructs.h>
#include <include/Tools/Transformation.h>

constexpr char ROOT_PATH[] = "../MultiAgentSlam-ROS/src/slam_logger/logs";
constexpr char ERR_FILE[] = "/errors.txt";
constexpr char BEL_FILE[] = "/beliefs.txt";
constexpr char FEAT_FILE[] = "/features.txt";
constexpr char TRANS_FILE[] = "/transformations.txt";

class Logger {
public:
    static Logger *getInstance(const std::string &rName) {
        static Logger instance(rName);
        return &instance;
    }
    ~Logger() = default;
    Logger(const Logger &) = delete;
    void operator=(const Logger &) = delete;

    void report_err(const std::string &message);
    void record(const BELIEF &belief);
    void record(const std::array<FEATURE, FEATURE_LIMIT> &features, const CLASSIFIER &classifer);
    void record(const POSE &transformation, const std::string &targetRover);
    void clearErrorLog();
    void clearInfoLogs();

private:
    Logger(std::string rName) :
        roverName(std::move(rName))
    {
        std::cout << "Creating Logger for " << roverName << "..." << std::endl;

        boost::filesystem::path dir(ROOT_PATH);
        if (!boost::filesystem::exists(dir)) {
            if (boost::filesystem::create_directory(dir)) {
                std::cout << "Directory <" << dir << "> created." << std::endl;
            }
        }

        // Create rover specific directory
        dir += "/" + roverName;
        if (!boost::filesystem::exists(dir)) {
            if (boost::filesystem::create_directory(dir)) {
                std::cout << "Directory <" << dir << "> created." << std::endl;
            }
        }
        errPath << dir.string() << ERR_FILE;
        belPath << dir.string() << BEL_FILE;
        featPath << dir.string() << FEAT_FILE;
        transPath << dir.string() << TRANS_FILE;

        this->eLog.open(errPath.str(), std::ofstream::out);
        this->beliefLog.open(belPath.str(), std::ofstream::out);
        this->featureLog.open(featPath.str(), std::ofstream::out);
        this->transformationLog.open(transPath.str(), std::ofstream::out);

        this->beliefLog << "time, rover, x, y, theta, confidence" << std::endl;
        this->featureLog << "time, x_1, y_1, incid_1, x_2, y_2, incid_2, x_3, y_3, incid_3, area, orientation, signature" << std::endl;
        this->transformationLog << "time, curr, targ, x, y, theta" << std::endl;

        ticks = clock();
    }

    long getTime();
    void writeToFile(std::ofstream &file, const std::string &message);

    std::string roverName;

    std::stringstream errPath;
    std::stringstream belPath;
    std::stringstream featPath;
    std::stringstream transPath;

    std::ofstream eLog;
    std::ofstream beliefLog;
    std::ofstream featureLog;
    std::ofstream transformationLog;
    clock_t ticks;
};

#endif //MULTIAGENTSLAM_LOGGER_H
