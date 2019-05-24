//
// Created by Mason U'Ren on 2019-02-14.
//

#include "ConfigParser.h"

// for convenience
using json = nlohmann::json;

void ConfigParser::parseConfig(SYS_CONFIG_IN *in, json *dataPtr) {
    std::hash<nlohmann::json> hasher;
    json data = *dataPtr;

    JSON_CONFIG *config = &in->config;
    SLAM_CONFIG *slamConfig;
    ROVER_CONFIG *roversConfig;
    SEIF_CONFIG *seifConfig;
    DETECTION_CONFIG *detectionConfig;
    LOCAL_MAP_CONFIG *localMapConfig;

    config->hash = hasher(data);

    try {
        slamConfig = &config->slamConfig;
        if (!data["slamConfig"].is_null()) {
            slamConfig->valid = true;
            slamConfig->numberOfRovers = data["slamConfig"]["numberOfRovers"].get<int>();
            slamConfig->filterSize = data["slamConfig"]["filterSize"].get<int>();
            slamConfig->maxFeatures = data["slamConfig"]["maxFeatures"].get<unsigned long>();

            detectionConfig = &(slamConfig->detectionConfig);
            if (!data["slamConfig"]["detection"].is_null()) {
                detectionConfig->valid = true;
                detectionConfig->highDetectionBoundaryInM =
                        data["slamConfig"]["detection"]["highDetectionBoundaryInM"].get<float>();
                detectionConfig->sonarCoverageInRad =
                        data["slamConfig"]["detection"]["sonarCoverageInRad"].get<float>();
                detectionConfig->sonarRangeInM =
                        data["slamConfig"]["detection"]["sonarRangeInM"].get<float>();
            }
            else {
                detectionConfig->valid = false;
            }
            seifConfig = &(slamConfig->seifConfig);
            if (!data["slamConfig"]["seif"].is_null()) {
                seifConfig->valid = true;
                seifConfig->maxActiveFeatures =
                        data["slamConfig"]["seif"]["maxActiveFeatures"].get<int>();
                seifConfig->featureDistInM = data["slamConfig"]["seif"]["featureDistInM"].get<float>();
                for (int i = 0; i < ELEMENT_SIZE; i++) {
                    seifConfig->R[i] = data["slamConfig"]["seif"]["R"][i].get<float>();
                    seifConfig->Q[i] = data["slamConfig"]["seif"]["Q"][i].get<float>();
                }
                seifConfig->maxFeatures = slamConfig->maxFeatures;
            }
            else {
                seifConfig->valid = false;
            }
            localMapConfig = &(slamConfig->localMapConfig);
            if (!data["slamConfig"]["localMap"].is_null()) {
                localMapConfig->valid = true;
                localMapConfig->featureSetML = data["slamConfig"]["localMap"]["featureSetML"].get<float>();
                localMapConfig->maxFeatures = slamConfig->maxFeatures;
            }
            else {
                localMapConfig->valid = false;
            }
            for (auto rover_id = 0; rover_id < slamConfig->numberOfRovers; rover_id++) {
                roversConfig = &(slamConfig->rovers[rover_id]);
                if (!data["slamConfig"]["rovers"][rover_id].is_null()) {
                    roversConfig->valid = true;
                    roversConfig->name = data["slamConfig"]["rovers"][rover_id]["name"].get<std::string>();
                }
                else {
                    roversConfig->valid = false;
                }
            }
        }
        else  {
            slamConfig->valid = false;
        }

    } catch (nlohmann::json::parse_error &e) {
        std::cout << e.what() << std::endl;
    }

}

bool ConfigParser::loadJSONFromFile(const std::string &filePath, json *dataPtr) {
    std::ifstream configFile(filePath);
    if (configFile.is_open()) {
        try {
            configFile >> *dataPtr;
            configFile.close();
            return true;
        } catch (nlohmann::detail::parse_error &e) {
            std::cout << e.what() << std::endl;
        }
    }
    return false;
}


