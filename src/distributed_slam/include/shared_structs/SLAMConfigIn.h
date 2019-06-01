//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SLAMCONFIGIN_H
#define C_SLAMCONFIGIN_H

#include <string>

/**
 * Structs
 */

typedef struct {
    bool valid{};
    int maxActiveFeatures{};
    unsigned long maxFeatures{};
    float featureDistInM{};
    float R[3]{};
    float Q[3]{};
} SEIF_CONFIG;

typedef struct {
    bool valid{};
    float highDetectionBoundaryInM{};
    float sonarCoverageInRad{};
    float sonarRangeInM{};
} DETECTION_CONFIG;

typedef struct {
    bool valid{};
    float featureSetML{};
    unsigned long maxFeatures{};
} LOCAL_MAP_CONFIG;

typedef struct {
    bool valid{};
    int ID{};
    std::string name;
} ROVER_CONFIG;

typedef struct {
    bool valid{};
    int numberOfRovers{};
    int filterSize{};
    unsigned long maxFeatures{};
    LOCAL_MAP_CONFIG localMapConfig{};
    DETECTION_CONFIG detectionConfig{};
    SEIF_CONFIG seifConfig{};
    ROVER_CONFIG rovers[6];
} SLAM_CONFIG;

typedef struct {
    SLAM_CONFIG slamConfig{};
    size_t hash{};
} JSON_CONFIG;




#endif //C_SLAMCONFIGIN_H
