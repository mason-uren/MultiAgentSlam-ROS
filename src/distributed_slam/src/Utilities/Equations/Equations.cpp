//
// Created by Mason U'Ren on 2019-02-21.
//

#include <limits>
#include "Equations.h"

// RAY - range (0) angle (1)
// POSE - x (0) y (1) theta (2)
Pose Equations::originToPoint(const Ray &ray,
        const Pose &pose,
        const bool &orthogonal) {
    return Pose{
            static_cast<float>((ray.range * sin(pose.theta + ray.angle)) + (orthogonal ? pose.y : pose.x)),
            static_cast<float>((ray.range * cos(pose.theta + ray.angle)) + (orthogonal ? pose.x : pose.y)),
            std::numeric_limits<float>::min()
    };
}

float Equations::wrapTheta(const float &orientation) {
    if (!this->isZero(orientation)) {
        if (M_PI - fabs(orientation) < 0) {
            float orient;
            if (this->isZero(orient = atan2(sin(orientation), cos(orientation)))) {
                return 0;
            }
            return orient;
        }
        return orientation;
    }
    return 0;
}

float Equations::normalizeValue(const float &value, const float &lowbound, const float &highbound) {
    return (value - lowbound) / (highbound - lowbound);
}

Location Equations::centroid(const std::array<Pose, 3> &coordinatePairs) {
    return Location {
            (coordinatePairs[0].x + coordinatePairs[1].x + coordinatePairs[2].x) / 3,
            (coordinatePairs[0].y + coordinatePairs[1].y + coordinatePairs[2].y) / 3
    };
}

// TODO : rename this function to better reflect equation type
// Szudzik's function
Signature Equations::szudzikMapping(const float &val_1, const float &val_2) {
//    return val_2 + (val_1 + val_2) * (val_1 + val_2 + 1) / 2;;
    std::vector<float> pt_pair{
        val_1, val_2,
        this->projectionEstimate(val_1, val_2)
    };
    
    return Signature(
            pt_pair.at(0),
            this->genBoundEllipse(pt_pair)
    );
}

float Equations::straightAvg(const std::vector<float> &toAvg) {
    float total = 0;
    for (auto value : toAvg) {
        total += value;
    }
    return total / toAvg.size();
}

float Equations::dotProduct(const std::vector<float> *vec_1, const std::vector<float> *vec_2) {
    float dotProduct = 0;
    if (vec_1 && vec_2) {
        for (unsigned long i = 0; i < vec_1->size(); i++) {
            dotProduct += (*vec_1)[i] * (*vec_2)[i];
        }
    }
    return dotProduct;
}

float Equations::distBetweenPts(const Pose &pose, const Pose &other) {
    return hypotf(pose.x - other.x, pose.y - other.y);
}

float Equations::distBetweenPts(const float &pt_a, const float &pt_b) {
    return sqrtf(powf(pt_a, 2) - powf(pt_b, 2));
};

bool Equations::isZero(const float &value) {
    return std::fabs(value) < 1E-4;
}

float Equations::mapSparcity(const float &sparcity) {
    static float distanceBetweenPts{};
    if (distanceBetweenPts < 0) {
        distanceBetweenPts = sparcity;
    }
    return distanceBetweenPts;
}

float Equations::projectionEstimate(const float &x, const float &y) {
    auto A{x >= 0 ? 2 * x : -2 * (x - 1)};
    auto B{y >= 0 ? 2 * y : -2 * (y - 1)};
    auto C{(A >= B ? powf(A, 2) + (A + B) : A + powf(B, 2)) / 2};
    return (x < 0 && y < 0) || (x >= 0 && y >= 0) ? C : -C - 1;
}

std::vector<float> Equations::genBoundEllipse(const std::vector<float> &pair) {
    std::vector<float> scaledBounds{pair.at(2), pair.at((2))};
    std::vector<float> boundsToScale{this->mapSparcity(), (-1) * this->mapSparcity()};
    for (auto xBound : boundsToScale) {
        for (auto yBound : boundsToScale) {
            auto pt{this->projectionEstimate((pair.at(0) + xBound), (pair.at(1) + yBound))};
            scaledBounds.at(0) = std::fmin(pt, scaledBounds.at(0));
            scaledBounds.at(1) = std::fmax(pt, scaledBounds.at(1));
        }
    }
    return scaledBounds;
}