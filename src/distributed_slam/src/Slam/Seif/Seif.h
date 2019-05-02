//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SEIF_H
#define C_SEIF_H

#include <array>
#include <vector>
#include <functional>
#include <iostream>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <future>         // std::async, std::future
#include <Eigen/Dense>

#include <include/SharedMemoryStructs.h>
#include <include/SLAMConfigIn.h>
#include <include/Matrix/Matrix.h>
#include <include/Tools/cpp14_utils.h>

#include "../../Agent/Moments/Moments.h"
#include "../../Utilities/Equations/Equations.h"
#include "../../Slam/FeatureSet/FeatureSet.h"

using std::unique_ptr;
using std::make_unique;

enum relation {
    EQUIV = 0,
    LOWER,
    HIGHER,
};

inline constexpr uint16_t featIdx(const unsigned long &idx) { return 3 * idx + 3; }

class Seif {
public:
    // Dummy constructor
//    Seif() : N(0) {} // Not sure why N "must" be initialized


    explicit Seif(SEIF_CONFIG *seifConfig) :
        N(ELEMENT_SIZE + (ELEMENT_SIZE * seifConfig->maxFeatures)),
        featuresFound(0),
        maxFeatures(seifConfig->maxFeatures),
        maxActiveFeatures(seifConfig->maxActiveFeatures),
        minFeatureDist(seifConfig->featureDistInM),
        maxCorrespondence(Equations::getInstance()->cantor(
                (seifConfig->featureDistInM * 2) * maxFeatures,
                (seifConfig->featureDistInM * 2)) * maxFeatures), // Diameter of feature marker
        recordedFeatures(new std::vector<FEATURE>(seifConfig->maxFeatures, FEATURE{})),
        activeFeatures(new std::vector<FEATURE>((u_long) maxActiveFeatures, FEATURE{})),
        toDeactivate(new FEATURE{}),
        informationMatrix(new Matrix<>(N, N)),
        informationVector(new Matrix<>(N)),
        stateEstimate(new Matrix<>(N)),
        motionCov(new Matrix<>(ELEMENT_SIZE, ELEMENT_SIZE)),
        measurementCov(new Matrix<>(ELEMENT_SIZE, ELEMENT_SIZE)),
        F_X(new Matrix<>(ELEMENT_SIZE, N)),
        delta(new Matrix<>(ELEMENT_SIZE)),
        del(new Matrix<>(ELEMENT_SIZE, ELEMENT_SIZE)),
        psi(new Matrix<>(N, N)),
        lambda(new Matrix<>(N, N)),
        phi(new Matrix<>(N, N)),
        kappa(new Matrix<>(N, N)),
        F_I(new Matrix<>((ELEMENT_SIZE - 1), N)),
        deltaPosition(new Matrix<>(ELEMENT_SIZE - 1)),
        q(0),
        zHat(new Matrix<>(ELEMENT_SIZE)),
        H(new Matrix<>(ELEMENT_SIZE, N))
    {
        // Init
        for (size_t i = 0; i < ELEMENT_SIZE; i++) {
            (*F_X)[i][i] = 1;
            (*motionCov)[i][i] = seifConfig->R[i];
            (*measurementCov)[i][i] = seifConfig->Q[i];
            // Need to mark <x, y, theta> as observed
            (*informationMatrix)[i][i] = 1;
        }
        toDeactivate->correspondence = std::numeric_limits<float>::min();
    }
    ~Seif() = default;

    void motionUpdate(const VELOCITY &velocity);
    POSE stateEstimateUpdate();
    void measurementUpdate(const RAY &incidentRay);
    void sparsification();
    POSE getRoverPose();
    void printRoverPose();

private:
    /**
     * Functions
     */
    // For testing purposes only
    bool printMatrices = false;

    // Motion Update (func)
    void updateDeltaDel(const VELOCITY &velocity);
    void updatePsi();
    void updateLambda();
    void updatePhi();
    void updateKappa();
    void updateOmegaBar();
    void updateEpsilonBar();
    void updateMuBar();

    // State Estimate (func)
    void integrateActiveFeatures();
    void generateStateEstimate(const Matrix<> *stateEstimate);

    // Measurement Update (func)
    bool isNewFeature(const RAY &incidentRay);
    void deriveFeature(FEATURE &feature, const RAY &incidentRay);
    bool hasBeenObserved(const float &correspondence);
    void addFeature(FEATURE &feature);
    uint16_t &nextFeatureIndex();
    void organizeFeatures();
    relation comparison(const float &identifier, const float &otherID);
    bool isActiveFull();
    void updateDeltaPos(const POSE &featPose);
    void update_q();
    void updateZHat(const float &correspondence);
    void updateH(const uint16_t &idx);
    void infoVecSummation(const FEATURE &feature);
    void infoMatrixSummation();

    // Sparsification (func)
    void updateInformationMatrix();
    void updateInformationVector(const Matrix<> *prevInfoMat);
    Matrix<> resolveProjection(const Matrix<> *projection);
    Matrix<> defineProjection(const FEATURE *feat, const bool &includePose = true);
    void makeInactive(FEATURE *toDeact);

    // Tools
    static bool correspondenceSort(const FEATURE &feat, const FEATURE &other);
    static bool distanceSort(const FEATURE &featA, const FEATURE &featB);

    /**
     * Variables
     */
    static POSE rPose;

    const uint16_t N;
    uint16_t featuresFound;
    uint16_t maxFeatures;
    int maxActiveFeatures;
    float minFeatureDist;
    float maxCorrespondence;
    unique_ptr<std::vector<FEATURE>> recordedFeatures;
    unique_ptr<std::vector<FEATURE>> activeFeatures;
    unique_ptr<FEATURE> toDeactivate;
    unique_ptr<Matrix<>> informationMatrix;
    unique_ptr<Matrix<>> informationVector;
    unique_ptr<Matrix<>> stateEstimate;

    unique_ptr<Matrix<>> motionCov;
    unique_ptr<Matrix<>> measurementCov;

    // Motion Update (vars)
    unique_ptr<Matrix<>> F_X;
    unique_ptr<Matrix<>> delta;
    unique_ptr<Matrix<>> del;
    unique_ptr<Matrix<>> psi;
    unique_ptr<Matrix<>> lambda;
    unique_ptr<Matrix<>> phi;
    unique_ptr<Matrix<>> kappa;

    // State Estimate (vars)
    unique_ptr<Matrix<>> F_I;

    // Measurment (vars)
    unique_ptr<Matrix<>> deltaPosition;
    float q;
    unique_ptr<Matrix<>> zHat;
    unique_ptr<Matrix<>> H;
};

#endif //C_SEIF_H
