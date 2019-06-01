//
// Created by Mason U'Ren on 2019-02-21.
//

#ifndef MULTIAGENTSLAM_EQUATIONS_H
#define MULTIAGENTSLAM_EQUATIONS_H


#include <array>
#include <vector>
#include <numeric>
#include <math.h>

#include <shared_structs/SharedMemoryStructs.h>

class Equations {
public:
    static Equations *getInstance() {
        static Equations instance;
        return &instance;
    }
    Equations(Equations const&) = delete;
    void operator=(Equations const&) = delete;

    Pose originToPoint(const Ray &ray,
            const Pose &pose,
            const bool &orthogonal = false);
    float wrapTheta(const float &orientation);
    float normalizeValue(const float &value, const float &lowbound, const float &highbound);
    Location centroid(const std::array<Pose, 3> &coordinatePairs);
    Signature szudzikMapping(const float &val_1, const float &val_2);
    float straightAvg(const std::vector<float> &toAvg);
    float dotProduct(const std::vector<float> *vec_1, const std::vector<float> *vec_2);
    float distBetweenPts(const Pose &pose, const Pose &other);
    float distBetweenPts(const float &pt_a, const float &pt_b);
    bool isZero(const float &value);
    
    float mapSparcity(const float &sparcity = 0.0f);

private:
    Equations () = default;

    float projectionEstimate(const float &x, const float &y);
    std::vector<float> genBoundEllipse(const std::vector<float> &pair);
    
};


#endif //MULTIAGENTSLAM_EQUATIONS_H
