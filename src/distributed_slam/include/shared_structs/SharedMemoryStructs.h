//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SHAREDSTRUCT_H
#define C_SHAREDSTRUCT_H

#include <algorithm>
#include <vector>
#include <iostream>

#include <better_enum/enum.h>

#include "SLAMConfigIn.h"

constexpr uint16_t FILTER_LENGTH = UINT16_C(64);
constexpr uint16_t RANGE_SENSOR_COUNT = UINT16_C(3);
constexpr uint16_t FEATURE_LIMIT = UINT16_C(3);
constexpr uint16_t SONAR_LIMIT = UINT16_C(3);
constexpr uint16_t SONAR_MAX_RANGE = UINT16_C(3);
//constexpr uint16_t SIGNATURE_MAX = UINT16_C(100);
constexpr uint16_t ELEMENT_SIZE = UINT16_C(3);
constexpr float ROS_INTERVAL = 0.1;
constexpr int BAD_ROVER_IDX = -1;


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
 * C++ "Structs"
 */
class Signature {
public:
    Signature(const float &pt,
              const std::vector<float> &bounds) :
          point(pt),
          bounds(bounds)
    {}
    Signature() = default;
    ~Signature() = default;

    float point{};
    std::vector<float> bounds{};

    Signature& operator=(const Signature &rhs) {
        point = rhs.point;
        bounds = rhs.bounds;
        return *this;
    }
    bool operator==(const Signature &rhs) const {
        return (point == rhs.point) &&
               (bounds == rhs.bounds);
    }
};

class Classifier {
public:
    Classifier(const float &area,
               const float &orientation,
               const Signature &signature) :
        area(area),
        orientation(orientation),
        signature(signature)
    {}
    Classifier() = default;
    ~Classifier() = default;

    float area{};
    float orientation{};
    Signature signature{};

    Classifier& operator=(const Classifier &rhs) {
        orientation = rhs.orientation;
        signature = rhs.signature;
        return *this;
    }
    bool operator==(const Classifier &rhs) {
        return (area == rhs.area) &&
               (orientation == rhs.orientation) &&
               (signature == rhs.signature);
    }
};

class Sonar {
public:
    Sonar() = default;
    Sonar(const sonar_id &id,
          const float &observedRange) :
        id(id),
        observedRange(observedRange)
    {}
    sonar_id id{};
    float observedRange{};

    Sonar& operator=(const Sonar &rhs) {
        id = rhs.id;
        observedRange = rhs.observedRange;
        return *this;
    }
    bool operator==(const Sonar &rhs) const {
        return (id == rhs.id) && (observedRange == rhs.observedRange);
    }
};

class Ray {
public:
    Ray() = default;
    Ray(const float &range,
        const float &angle) :
        range(range),
        angle(angle)
    {}
    float range{};
    float angle{};

    Ray& operator=(const Ray &rhs) {
        range = rhs.range;
        angle = rhs.angle;
        return *this;
    }
    bool operator==(const Ray &rhs) const {
        return (range == rhs.range) && (angle == rhs.angle);
    }
};

class Velocity {
public:
    Velocity() = default;
    Velocity(const float &linear,
             const float &angular) :
         linear(linear),
         angular(angular)
    {}
    float linear{};
    float angular{};

    Velocity& operator=(const Velocity &rhs) {
        linear = rhs.linear;
        angular = rhs.angular;
        return *this;
    }
    bool operator==(const Velocity &rhs) const {
        return (linear == rhs.linear) && (angular == rhs.angular);
    }
};

class Location {
public:
    Location() = default;
    Location(const float &x,
             const float &y) :
        x(x), y(y)
    {}
    float x{};
    float y{};

    Location& operator=(const Location &rhs) {
        x = rhs.x;
        y = rhs.y;
        return *this;
    }
    bool operator==(const Location &rhs) const {
        return (x == rhs.x) && (y == rhs.y);
    }
};

class Pose {
public:
    Pose() = default;
    Pose(const float &x,
         const float &y,
         const float &theta) :
        x(x),
        y(y),
        theta(theta)
    {}
    float x{};
    float y{};
    float theta{};

    Pose& operator=(const Pose &rhs) {
        x = rhs.x;
        y = rhs.y;
        theta = rhs.theta;
        return *this;
    }
    bool operator==(const Pose &rhs) const {
        return (x == rhs.x) && (y == rhs.y) && (theta == rhs.theta);
    }
};

class Belief {
public:
    Belief() = default;
    Belief(const Pose &currentPose,
           const float & roverConfidence) :
        currentPose(currentPose),
        roverConfidence(roverConfidence)
    {}
    Pose currentPose{};
    float roverConfidence{};

    Belief& operator=(const Belief &rhs) {
        currentPose = rhs.currentPose;
        roverConfidence = rhs.roverConfidence;
        return *this;
    }
    bool operator==(const Belief &rhs) const {
        return (currentPose == rhs.currentPose) & (roverConfidence == rhs.roverConfidence);
    }
};

class Feature {
public:
    Feature() = default;
    Feature(const uint16_t &idx,
            const Signature &correspondence,
            const Ray &incidentRay,
            const Pose &pose) :
        idx(idx),
        correspondence(correspondence),
        incidentRay(incidentRay),
        pose(pose)
    {}
    uint16_t idx{}; // TODO I don't like this here (doesn't relate to feature)
    Signature correspondence{};
    Ray incidentRay{};
    Pose pose{};

    Feature& operator=(const Feature &rhs) {
        idx = rhs.idx;
        correspondence = rhs.correspondence;
        incidentRay = rhs.incidentRay;
        pose = rhs.pose;
        return *this;
    }
    bool operator==(const Feature &rhs) const {
        return (correspondence == rhs.correspondence) && (incidentRay == rhs.incidentRay) && (pose == rhs.pose);
    }
};

typedef struct {
    JSON_CONFIG config;
    long block_id{};
} SYS_CONFIG_IN;

/**
 * Helper Functions (Global)
 */

template<typename Enum>
constexpr typename std::underlying_type<Enum>::type num(const Enum &anEnum) noexcept {
    return static_cast<typename std::underlying_type<Enum>::type>(anEnum);
};

inline std::string & upper(std::string &name) {
    std::transform(name.begin(), name.end(), name.begin(), ::toupper);
    return name;
}

inline std::string & lower(std::string &name) {
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    return name;
}

inline int getRoverAddress(const std::string &name) {
    if (!name.empty()) {
        auto copy{name};
        return rover_names::_from_string(upper(copy).c_str());
    }
    return BAD_ROVER_IDX;
}

inline std::string getRoverName(const uint16_t &idx) {
    std::string rName{rover_names::_from_integral(idx)._to_string()};
    return lower(rName);
}

#endif //C_SHAREDSTRUCT_H
