//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SHAREDSTRUCT_H
#define C_SHAREDSTRUCT_H

#include <include/Tools/enum.h>
#include <algorithm>

#include "SLAMConfigIn.h"

constexpr uint16_t FILTER_LENGTH = UINT16_C(64);
constexpr uint16_t RANGE_SENSOR_COUNT = UINT16_C(3);
constexpr uint16_t FEATURE_LIMIT = UINT16_C(3);
constexpr uint16_t SONAR_LIMIT = UINT16_C(3);
constexpr uint16_t SIGNATURE_MAX = UINT16_C(100);
constexpr uint16_t ELEMENT_SIZE = UINT16_C(3);
constexpr float ROS_INTERVAL = 0.1;


/**
 * Enumerations
 */

enum class pos_val {
    X = 0,
    Y,
    THETA
};
enum class measurement {
    RANGE = 0,
    ANGLE = 1,
    CORRESPONDENCE = 2
};

enum class cov_idx {
    XY = 0,
    XZ = 1,
    YZ = 2,
};

enum class node_color {
    BLACK = 0,
    RED
};

enum class sonar_id {
    S_LEFT = 0,
    S_CENTER,
    S_RIGHT
};

BETTER_ENUM(rover_names, uint16_t,
    ACHILLES = 0,
    AENEAS,
    AJAX,
    DIOMEDES,
    HECTOR,
    PARIS
)

/**
 * Structs
 */

typedef struct Classifier {
    float area;
    float orientation;
    float signature;

    void operator=(const Classifier &rhs) {
        area = rhs.area;
        orientation = rhs.orientation;
        signature = rhs.signature;
    }
    bool operator==(const Classifier &rhs) const {
        return (area == rhs.area) && (orientation == rhs.orientation) && (signature == rhs.signature);
    }
} CLASSIFIER;

typedef struct Sonar {
    sonar_id id;
    float observedRange;

    void operator=(const Sonar &rhs) {
        id = rhs.id;
        observedRange = rhs.observedRange;
    }
    bool operator==(const Sonar &rhs) const {
        return (id == rhs.id) && (observedRange == rhs.observedRange);
    }
} SONAR;

typedef struct Ray {
    float range;
    float angle;

    void operator=(const Ray &rhs) {
        range = rhs.range;
        angle = rhs.angle;
    }
    bool operator==(const Ray &rhs) const {
        return (range == rhs.range) && (angle == rhs.angle);
    }
} RAY;

typedef struct Velocity {
    float linear;
    float angular;

    void operator=(const Velocity &rhs) {
        linear = rhs.linear;
        angular = rhs.angular;
    }
    bool operator==(const Velocity &rhs) const {
        return (linear == rhs.linear) && (angular == rhs.angular);
    }
} VELOCITY;

typedef struct Location {
    float x;
    float y;

    void operator=(const Location &rhs) {
        x = rhs.x;
        y = rhs.y;
    }
    bool operator==(const Location &rhs) const {
        return (x == rhs.x) && (y == rhs.y);
    }
} LOCATION;

typedef struct Pose {
    float x;
    float y;
    float theta;

    void operator=(const Pose &rhs) {
        x = rhs.x;
        y = rhs.y;
        theta = rhs.theta;
    }
    bool operator==(const Pose &rhs) const {
        return (x == rhs.x) && (y == rhs.y) && (theta == rhs.theta);
    }
} POSE;

typedef struct Belief {
    POSE currentPose;
    float roverConfidence;

    void operator=(const Belief &rhs) {
        currentPose = rhs.currentPose;
        roverConfidence = rhs.roverConfidence;
    }
    bool operator==(const Belief &rhs) const {
        return (currentPose == rhs.currentPose) & (roverConfidence == rhs.roverConfidence);
    }
} BELIEF;

typedef struct Feature {
    uint16_t idx{}; // TODO I don't like this here (doesn't relate to feature)
    float correspondence{};
    RAY incidentRay{};
    POSE pose{};

    void operator=(const Feature &rhs) {
        idx = rhs.idx;
        correspondence = rhs.correspondence;
        incidentRay = rhs.incidentRay;
        pose = rhs.pose;
    }
    bool operator==(const Feature &rhs) const {
        return (correspondence == rhs.correspondence) && (incidentRay == rhs.incidentRay) && (pose == rhs.pose);
    }
} FEATURE;

typedef struct {
    JSON_CONFIG config;
    long block_id{};
} SYS_CONFIG_IN;

template<typename Enum>
constexpr typename std::underlying_type<Enum>::type num(const Enum &anEnum) noexcept {
    return static_cast<typename std::underlying_type<Enum>::type>(anEnum);
};

inline uint16_t getRoverAddress(const std::string &name) {
    auto copy{name};
    std::transform(copy.begin(), copy.end(), copy.begin(), ::toupper);
    return rover_names::_from_string(copy.c_str());
}

inline std::string getRoverName(const uint16_t &idx) {
    return rover_names::_from_integral(idx)._to_string();
}

#endif //C_SHAREDSTRUCT_H
