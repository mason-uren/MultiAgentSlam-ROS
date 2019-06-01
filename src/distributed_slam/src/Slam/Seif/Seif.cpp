//
// Created by Mason U'Ren on 2019-02-13.
//

#include "Seif.h"

using lim_float = std::numeric_limits<float>;

// IMPORTANT: should not be accessed/used outside of Seif
Pose Seif::rPose = Pose{};

void Seif::printRoverPose() {
    std::cout << "R-Pose : (" <<
        Seif::rPose.x << ", " <<
        Seif::rPose.y << ", " <<
        Seif::rPose.theta << ")" << std::endl;
}

void Seif::motionUpdate(const Velocity &velocity) {
    if (this->printMatrices) {
        std::cout << "F_X:" << std::endl;
        this->F_X->print();
    }

    this->updateDeltaDel(velocity);
    this->updatePsi();
    this->updateLambda();
    this->updatePhi();
    this->updateKappa();
    this->updateOmegaBar();
    this->updateEpsilonBar();
    this->updateMuBar();
}

Pose Seif::stateEstimateUpdate() {
    auto stateEstimate(*this->stateEstimate);
    this->integrateActiveFeatures();
    this->generateStateEstimate(&stateEstimate);
    rPose = {
        Equations::getInstance()->isZero(
                this->stateEstimate->at(num(pos_val::X))) ?
                        0 : this->stateEstimate->at(num(pos_val::X)),
        Equations::getInstance()->isZero(
                this->stateEstimate->at(num(pos_val::Y))) ?
                        0 : this->stateEstimate->at(num(pos_val::Y)),
        Equations::getInstance()->wrapTheta(this->stateEstimate->at(num(pos_val::THETA)))
    };
    return rPose;
}

// Must always run regardless of valid detection. Found features still need to
// updated with new rover estimate.
void Seif::measurementUpdate(const Ray &incidentRay) {
    auto feature(Feature{});
    // Bad Feautre Struct : {range = -MAXFLOAT, angle = -MAXFLOAT}
    if (this->isNewFeature(incidentRay)) {
        this->deriveFeature(feature, incidentRay);
        if (!this->hasBeenObserved(feature.correspondence)) {
            FeatureSet::getInstance()->addToSet(feature, rPose);
            this->addFeature(feature);
            this->organizeFeatures();
        }
    }

    for (size_t featIdx = 0; featIdx < this->featuresFound; featIdx++) {
        feature = (*this->recordedFeatures)[featIdx];
        this->updateDeltaPos(feature.pose);
        this->update_q();
        this->updateZHat(feature.correspondence);
        this->updateH(featIdx);
        this->infoVecSummation(feature);
        this->infoMatrixSummation();
    }
}

void Seif::sparsification() {
    auto infoMat(*this->informationMatrix);
    this->updateInformationMatrix();
    this->updateInformationVector(&infoMat);
}

void Seif::updateDeltaDel(const Velocity &velocity) {
    float ratio;
    auto thetaPrior(rPose.theta);
    auto theta((float) (velocity.angular * ROS_INTERVAL));
    if (velocity.angular != 0) {
        ratio = velocity.linear / velocity.angular;
        this->delta->at(num(pos_val::X)) = (-1) * ratio * sin(thetaPrior) + ratio * sin(thetaPrior + theta);
        this->delta->at(num(pos_val::Y)) = ratio * cos(thetaPrior) - ratio * cos(thetaPrior + theta);

        (*this->del)[num(pos_val::X)][2] = ratio * cos(thetaPrior) - ratio * cos(thetaPrior + theta);
        (*this->del)[num(pos_val::Y)][2] = ratio * sin(thetaPrior) - ratio * sin(thetaPrior + theta);
    }
    else {
        ratio = velocity.linear;
        this->delta->at(num(pos_val::X)) = ratio * sin(thetaPrior);
        this->delta->at(num(pos_val::Y)) = ratio * cos(thetaPrior);

        (*this->del)[num(pos_val::X)][2] = (-1) * ratio * cos(thetaPrior);
        (*this->del)[num(pos_val::Y)][2] = (-1) * ratio * sin(thetaPrior);
    }
    this->delta->at(num(pos_val::THETA)) = theta;

    if (this->printMatrices) {
        std::cout << "Delta: " << std::endl;
        this->delta->print();
        std::cout << "Del: " << std::endl;
        this->del->print();
    }
}

void Seif::updatePsi() {
    Matrix<float> idMat(this->del->numRows(), this->del->numRows()); idMat.identity();
    Matrix<float> fx_T(*this->F_X); fx_T.transpose();
    Matrix<float> inverse((idMat + *this->del)); inverse.invert();
    *this->psi = fx_T * (inverse - idMat) * *this->F_X;

    if (this->printMatrices) {
        std::cout << "PSI:" << std::endl;
        this->psi->print();
    }
}

void Seif::updateLambda() {
    auto psi_T(*this->psi); psi_T.transpose();
    *this->lambda =
            psi_T * *this->informationMatrix +
            *this->informationMatrix * *this->phi +
            psi_T * *this->informationMatrix * *this->phi;

    if (this->printMatrices) {
        std::cout << "Lambda:" << std::endl;
        this->lambda->print();
    }
}

void Seif::updatePhi() {
    *this->phi = *this->informationMatrix + *this->lambda;

    if (this->printMatrices) {
        std::cout << "PHI:" << std::endl;
        this->phi->print();
    }
}

void Seif::updateKappa() {
    auto fx_T(*this->F_X); fx_T.transpose();
    auto R_inv(*this->motionCov); R_inv.invert();
    auto inverse(R_inv + *this->F_X * *this->phi * fx_T); inverse.invert();
    *this->kappa = *this->phi * fx_T * inverse * *this->F_X * *this->phi;

    if (this->printMatrices) {
        std::cout << "Kappa:" << std::endl;
        this->kappa->print();
    }
}

void Seif::updateOmegaBar() {
    *this->phi -= *this->kappa;
    *this->informationMatrix = *this->phi;

    if (this->printMatrices) {
        std::cout << "Info Mat:" << std::endl;
        this->informationMatrix->print();
    }
}

void Seif::updateEpsilonBar() {
    auto fx_T(*this->F_X); fx_T.transpose();
    *this->informationVector +=
            ((*this->lambda - *this->kappa) * *this->stateEstimate +
            (*this->informationMatrix * fx_T * *this->delta));

    if (this->printMatrices) {
        std::cout << "Info Vec: " << std::endl;
        this->informationVector->print();
    }
}

void Seif::updateMuBar() {
    auto fx_T(*this->F_X); fx_T.transpose();
    *this->stateEstimate += fx_T * *this->delta;

    if (this->printMatrices) {
        std::cout << "State estimate: " << std::endl;
        this->stateEstimate->print();
    }
}

void Seif::integrateActiveFeatures() {
    uint16_t startIdx;
    this->F_I->zeroMatrix();
    auto resVec(static_cast<Matrix<float>>(this->informationVector->numRows()));

    for (long i = 0; i < std::fmin(this->featuresFound, this->maxActiveFeatures); i++) {
        Feature *feature = &((*this->activeFeatures)[i]);
        if (!feature) {
            break;
        }

        startIdx = featIdx(feature->idx);
        (*this->F_I)[num(pos_val::X)][startIdx] = 1;
        (*this->F_I)[num(pos_val::Y)][startIdx + num(pos_val::Y)] = 1;

        auto fi_T(*this->F_I); fi_T.transpose();
        auto inverse(*this->F_I * *this->informationMatrix * fi_T); inverse.invert();
        resVec =
                inverse * *this->F_I *
                (*this->informationVector - *this->informationMatrix * *this->stateEstimate +
                *this->informationMatrix * fi_T * *this->F_I * *this->stateEstimate);

        this->stateEstimate->at(startIdx) = resVec.at(num(pos_val::X));
        this->stateEstimate->at(startIdx + num(pos_val::Y)) = resVec.at(num(pos_val::Y));

        if (this->printMatrices) {
            std::cout << "State Estimate (Active):" << std::endl;
            this->stateEstimate->print();
        }
    }
}

void Seif::generateStateEstimate(const Matrix<float> *stateEstimate) {
    auto resVec(static_cast<Matrix<float>>(this->informationVector->numRows()));
    auto fx_T(*this->F_X); fx_T.transpose();
    auto inverse(*this->F_X * *this->informationMatrix * fx_T); inverse.invert();
    resVec =
            inverse * *this->F_X *
             (*this->informationVector - *this->informationMatrix * *stateEstimate +
              *this->informationMatrix * fx_T * *this->F_X * *stateEstimate);
    this->stateEstimate->at(num(pos_val::X)) = resVec.at(num(pos_val::X));
    this->stateEstimate->at(num(pos_val::Y)) = resVec.at(num(pos_val::Y));
    this->stateEstimate->at(num(pos_val::THETA)) = resVec.at(num(pos_val::THETA));

    if (this->printMatrices) {
        std::cout << "State Estimate (All):" << std::endl;
        this->stateEstimate->print();
    }
}

bool Seif::isNewFeature(const Ray &incidentRay) {
    return incidentRay.range != lim_float::min() && incidentRay.angle != lim_float::min();
}

void Seif::deriveFeature(Feature &feature, const Ray &incidentRay) {
    auto xyCoords(Equations::getInstance()->originToPoint(
            incidentRay,
            {rPose.x, rPose.y, rPose.theta},
            true)); // true

    feature.incidentRay = incidentRay;
    feature.pose = xyCoords;

    // TODO may need to think how we normalize value
    feature.correspondence = Equations::getInstance()->normalizeValue(
            Equations::getInstance()->cantor(xyCoords.x, xyCoords.y),
            0, this->maxCorrespondence
    );
}

/**
 * @fn hasBeenObserved
 * @brief Utilizes a binary search to check for if feature has already been obeserved.
 *
 * Observation is confirmed/denied depending on whether the distance between two features
 * falls below #minFeatureDist.
 * @param correspondence - the associated feature identifier
 * @return Was the currently viewed feature previously observed.
 */
bool Seif::hasBeenObserved(const float &correspondence) {
    uint16_t front = 0;
    uint16_t back = this->featuresFound ? this->featuresFound - 1 : 0;

    if (!this->featuresFound) {
        return false;
    }
    if (this->featuresFound < 2) {
        return EQUIV == comparison(correspondence, (*this->recordedFeatures)[front].correspondence);
    }

    bool finished = false;
    do {
        auto position((u_long) ((front && back) ? floor((front + back) / 2) : 0));
        std::cout << __PRETTY_FUNCTION__ << " : POSITION -> " << position << std::endl;
        auto equivalence(comparison(correspondence, (*this->recordedFeatures)[position].correspondence));
        switch (equivalence) {
            case EQUIV:
                return true;
            case LOWER:
                if (!position) {
                    finished = true;
                }
                back = --position;
                break;
            case HIGHER:
                front = ++position;
                break;
            default:
                std::cout << "Error: bad feature identifier comparison <" << equivalence << ">" << std::endl;
                exit(EXIT_FAILURE);
        }
    } while (front <= back && !finished);

    return false;
}

void Seif::addFeature(Feature &feature) {
    feature.idx = this->nextFeatureIndex();
    auto idx(feature.idx);
    if (this->isActiveFull()) {
        *this->toDeactivate = (*this->activeFeatures)[idx = 0]; // Keep the furthest away feature in the initial position;
    }
    (*this->activeFeatures)[idx] = feature;
    (*this->recordedFeatures)[this->nextFeatureIndex()++] = feature;
}

uint16_t &Seif::nextFeatureIndex() {
    if (this->featuresFound < this->maxFeatures) {
        return this->featuresFound;
    }
    perror("Maximum number of features observed. Consider allowing more observed features : 'maxFeatures'.");
    exit(EXIT_FAILURE);
}

void Seif::organizeFeatures() {
    if (this->featuresFound > 1) {
        auto endPtr_active((this->isActiveFull() ?
                              this->activeFeatures->end() :
                              this->activeFeatures->begin() + this->featuresFound));
        std::sort(this->activeFeatures->begin(), endPtr_active, distanceSort);

        auto endPtr_all(this->recordedFeatures->begin() + this->featuresFound);
        std::sort(this->recordedFeatures->begin(), endPtr_all, correspondenceSort);
    }
}

relation Seif::comparison(const float &identifier, const float &otherID) {
    return (abs(identifier - otherID) < minFeatureDist) ? EQUIV : ((identifier < otherID) ? LOWER : HIGHER);
}

bool Seif::isActiveFull() {
    return this->featuresFound >= this->maxActiveFeatures;
}

void Seif::updateDeltaPos(const Pose &featPose) {
    this->deltaPosition->at(num(pos_val::X)) = featPose.x - rPose.x;
    this->deltaPosition->at(num(pos_val::Y)) = featPose.y - rPose.y;
}

void Seif::update_q() {
    auto dp_T(*this->deltaPosition); dp_T.transpose();
    this->q = (dp_T * *this->deltaPosition).at(0);
}

void Seif::updateZHat(const float &correspondence) {
    this->zHat->at(num(measurement::RANGE)) = sqrt(this->q);
    this->zHat->at(num(measurement::ANGLE)) = atan2(
            this->deltaPosition->at(num(pos_val::Y)),
            this->deltaPosition->at(num(pos_val::X))) - rPose.theta;
    this->zHat->at(num(measurement::CORRESPONDENCE)) = correspondence;
}

void Seif::updateH(const uint16_t &idx) {
    (*this->H)[num(pos_val::X)][num(pos_val::X)] =
            sqrt(this->q) * this->deltaPosition->at(num(pos_val::X));
    (*this->H)[num(pos_val::X)][num(pos_val::Y)] =
            (-1) * sqrt(this->q) * this->deltaPosition->at(num(pos_val::Y));
    (*this->H)[num(pos_val::Y)][num(pos_val::X)] = this->deltaPosition->at(num(pos_val::Y));
    (*this->H)[num(pos_val::Y)][num(pos_val::Y)] = this->deltaPosition->at(num(pos_val::X));
    (*this->H)[num(pos_val::Y)][2] = -1;

    auto startIdx{featIdx(idx)};

    (*this->H)[num(pos_val::X)][startIdx + num(pos_val::X)] =
            (-1) * sqrt(this->q) * this->deltaPosition->at(num(pos_val::X));
    (*this->H)[num(pos_val::X)][startIdx + num(pos_val::Y)] =
            sqrt(this->q) * this->deltaPosition->at(num(pos_val::Y));
    (*this->H)[num(pos_val::Y)][startIdx + num(pos_val::X)] = (-1) * this->deltaPosition->at(num(pos_val::Y));
    (*this->H)[num(pos_val::Y)][startIdx + num(pos_val::Y)] = (-1) * this->deltaPosition->at(num(pos_val::X));
    (*this->H)[2][startIdx + 2] = 1;

    *this->H *= (1 / this->q);
}

void Seif::infoVecSummation(const Feature &feature) {
    Matrix<float> z_i{{feature.incidentRay.range}, {feature.incidentRay.angle}, {feature.correspondence}};
    auto H_T(*this->H); H_T.transpose();
    auto Q_inv(*this->measurementCov); Q_inv.invert();
    *this->informationVector += H_T * Q_inv * (z_i - *this->zHat - *this->H * *this->stateEstimate);
}

void Seif::infoMatrixSummation() {
    auto H_T(*this->H); H_T.transpose();
    auto Q_inv(*this->measurementCov); Q_inv.invert();
    *this->informationMatrix += H_T * Q_inv * *this->H;
}



void Seif::updateInformationMatrix() {
    auto m0_T(this->defineProjection(&(*this->toDeactivate), false)); m0_T.transpose();
    auto xm0_T(this->defineProjection(&(*this->toDeactivate))); xm0_T.transpose();
    auto fx_T(*this->F_X); fx_T.transpose();
    if (this->toDeactivate->correspondence >= 0) {
        this->makeInactive(&(*this->toDeactivate));
    }

    *this->informationMatrix -= this->resolveProjection(&m0_T) + this->resolveProjection(&xm0_T) - this->resolveProjection(&fx_T);
}

void Seif::updateInformationVector(const Matrix<float> *prevInfoMat) {
    *this->informationVector += (*this->informationMatrix - *prevInfoMat) * *this->stateEstimate;
}

Matrix<float> Seif::resolveProjection(const Matrix<float> *projection) {
    auto p_T(*projection); p_T.transpose();
    auto inverse(p_T * *this->informationMatrix * *projection); inverse.invert();
    return static_cast<Matrix<>>(*this->informationMatrix * *projection * inverse * p_T * *this->informationMatrix);
}

Matrix<float> Seif::defineProjection(const Feature *feat, const bool &includePose) {
    Matrix<float> projection(this->F_X->numRows(), this->F_X->numCols());
    if (includePose) {
        projection[num(pos_val::X)][num(pos_val::X)] = 1;
        projection[num(pos_val::Y)][num(pos_val::Y)] = 1;
        projection[num(pos_val::THETA)][num(pos_val::THETA)] = 1;
    }
    if (feat->correspondence >= 0) {
        auto startIdx{featIdx(feat->idx)};
        projection[num(pos_val::X)][startIdx] = 1;
        projection[num(pos_val::Y)][startIdx + num(pos_val::Y)] = 1;
        projection[num(measurement::CORRESPONDENCE)][startIdx + num(measurement::CORRESPONDENCE)] = 1;
    }

    return projection;
}

void Seif::makeInactive(Feature *toDeact) {
    toDeact->correspondence = lim_float::min();
}

/**
 * @fn correspondenceSort
 * @brief sort #FEATUREs based on their correspondence values.
 *
 * Sorts from lowest to highest.
 * @param feat reference to a feature
 * @param other reference to a different feaure
 * @return is the correspondence of #feat lower than #other.
 */
bool Seif::correspondenceSort(const Feature &feat, const Feature &other) {
    return feat.correspondence < other.correspondence;;
}

/**
 * @fn distanceSort
 * @brief sort #FEATUREs based on their relationship to the current rover position.
 *
 * Sorts from highest to lowest.
 * @param featA reference to a feature
 * @param featB reference to a feature
 * @return is the relative distance between rover and #featA larger than that with #featB.
 */
bool Seif::distanceSort(const Feature &featA, const Feature &featB) {
    auto distA(Equations::getInstance()->distBetweenPts(featA.pose, rPose));
    auto distB(Equations::getInstance()->distBetweenPts(featB.pose, rPose));
    return distA > distB;
}

Pose Seif::getRoverPose() {
    return rPose;
}
