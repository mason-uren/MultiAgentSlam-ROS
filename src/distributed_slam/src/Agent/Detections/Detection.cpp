//
// Created by Mason U'Ren on 2019-02-13.
//

#include "Detection.h"

bool Detection::hasIncidentRay() {
    return this->incidentRay->range < this->upperDetectLimit;
//    return this->incidentRay->range < 0.6;
}

Ray* Detection::getIncidentRay() {
    return &(*this->incidentRay);
}

void Detection::setIncidentRay(Ray ray) {
    *this->incidentRay = ray;
}

void Detection::MLIncidentRay(const std::array<Sonar, 3> &sonar) {
    for (auto ray : sonar) {
        this->addToFilter(ray);
    }

    this->inverseRayWeighting();
}

void Detection::addToFilter(const Sonar &ray) {
    float range =
        std::fmin(
                std::fmax(0, ray.observedRange),
                SONAR_MAX_RANGE
        );

//    float range = (ray.observedRange < 0) ? 0 : ray.observedRange;
    float filtered;
    switch (ray.id) {
        case sonar_id::S_LEFT:
            filtered = (*this->leftSonarFilter).filterValue(range);
            break;
        case sonar_id::S_CENTER:
            filtered = (*this->centerSonarFilter).filterValue(range);
            break;
        case sonar_id::S_RIGHT:
            filtered = (*this->rightSonarFilter).filterValue(range);
            break;
    }
}

void Detection::inverseRayWeighting() {
    float leftRange, centerRange, rightRange;
    float minRange = std::fmin(
            std::fmin(
                    leftRange = this->leftSonarFilter->getValue(),
                    centerRange = this->centerSonarFilter->getValue()
                    ),
            rightRange = this->rightSonarFilter->getValue());

    float resultRayX = 0, resultRayY = 0;
    std::array<float, 3> ranges {leftRange, centerRange, rightRange};
    for (auto ray = uint(0); ray < ranges.size(); ray++) {
        resultRayX += sin((*this->incidentAngles)[ray]) * (1 - (ranges[ray] / this->sonarRangeInM));
        resultRayY += cos((*this->incidentAngles)[ray]) * (1 - (ranges[ray] / this->sonarRangeInM));
    }
    this->setIncidentRay({minRange, atan2f(resultRayX, resultRayY)});
}
