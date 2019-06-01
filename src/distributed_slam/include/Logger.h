//
// Created by Mason U'Ren on 2019-04-25.
//

#ifndef MULTIAGENTSLAM_LOGGER_H
#define MULTIAGENTSLAM_LOGGER_H

#include <utility>
#include <iostream>
#include <array>
#include <fstream>
#include <mutex>
#include <sstream>
#include <fcntl.h>
#include <time.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

#include <shared_structs/SharedMemoryStructs.h>

constexpr char ROOT_PATH[] = "../MultiAgentSlam-ROS/src/distributed_slam/logs";
constexpr char ERR_FILE[] = "/errors.txt";
constexpr char STATUS_FILE[] = "/status.txt";
constexpr char BEL_FILE[] = "/beliefs.txt";
constexpr char FEAT_FILE[] = "/features.txt";
constexpr char TRANS_FILE[] = "/transformations.txt";

class Logger {
public:
    static Logger *getInstance(const std::string &rName = "default", const std::string &filePath = "") {
        static std::string roverName{};
        static std::string rootPath{};
        if (roverName.empty()) {
            roverName = rName;
        }
        if (rootPath.empty()) {
            rootPath = filePath;
        }
        static Logger instance(roverName, filePath);
        return &instance;
    }
    ~Logger() = default;
    Logger(const Logger &) = delete;
    void operator=(const Logger &) = delete;

    void error(const std::string &message);
    void status(const std::string &message);
    void record(const Belief &belief);
    void record(const std::array<Feature, FEATURE_LIMIT> &features, const Classifier &classifer);
    void record(const Pose &transformation, const std::string &targetRover);
    void clearErrorLog();
    void clearInfoLogs();
    std::string getLoggerName();

    // For testing environment ONLY
    void logTo(const std::string &filePath, const std::string &msg);

private:
    explicit Logger(std::string rName, std::string rPath) :
        roverName(std::move(rName)),
        rootPath(std::move(rPath))
    {
        std::cout << "Creating Logger for " << roverName << "..." << std::endl;

        boost::filesystem::path dir(rootPath + "/logs");
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
        statusPath << dir.string() << STATUS_FILE;
        belPath << dir.string() << BEL_FILE;
        featPath << dir.string() << FEAT_FILE;
        transPath << dir.string() << TRANS_FILE;

        this->eLog.open(errPath.str(), std::ofstream::out);
        this->sLog.open(statusPath.str(), std::ofstream::out);
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

    std::string rootPath;
    std::string roverName;

    std::stringstream errPath;
    std::stringstream statusPath;
    std::stringstream belPath;
    std::stringstream featPath;
    std::stringstream transPath;

    std::ofstream eLog;
    std::ofstream sLog;
    std::ofstream beliefLog;
    std::ofstream featureLog;
    std::ofstream transformationLog;
    clock_t ticks;
};

#endif //MULTIAGENTSLAM_LOGGER_H
