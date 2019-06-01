//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_DETECTION_H
#define C_DETECTION_H

#include <memory>
#include <array>

#include <include/IncidentRayInterface.h>
#include <include/SLAMConfigIn.h>
#include <include/Filters/FIRFilter.h>

typedef struct {
    float left[FILTER_LENGTH]{};
    float center[FILTER_LENGTH]{};
    float right[FILTER_LENGTH]{};
} SONAR_FILTER_BUFS;

class Detection : public IncidentRayInterface {
public:
    explicit Detection(DETECTION_CONFIG *detectionConfig) :
        upperDetectLimit(detectionConfig->highDetectionBoundaryInM),
        sonarRangeInM(detectionConfig->sonarRangeInM),
        incidentAngles(new std::array<float, RANGE_SENSOR_COUNT>
            {
                (detectionConfig->sonarCoverageInRad / 2),
                (0),
                (-detectionConfig->sonarCoverageInRad / 2)
            }),
        incidentRay(new Ray {detectionConfig->sonarRangeInM, 0}),
        buffers(new SONAR_FILTER_BUFS()),
        leftSonarFilter(new FIRFilter<float, FILTER_LENGTH>(buffers->left)),
        centerSonarFilter(new FIRFilter<float, FILTER_LENGTH>(buffers->center)),
        rightSonarFilter(new FIRFilter<float, FILTER_LENGTH>(buffers->right))
        {}

    ~Detection() override = default;

    bool hasIncidentRay() override; // TODO currently not being utilized
    Ray *getIncidentRay() override;

    void MLIncidentRay(const std::array<Sonar, 3> &sonar);

private:
    void setIncidentRay(Ray ray) override;

    void addToFilter(const Sonar &ray);
    void inverseRayWeighting();

    /**
     * Variables
     */
    float upperDetectLimit;
    float sonarRangeInM;
    std::shared_ptr<const std::array<float, RANGE_SENSOR_COUNT>> incidentAngles;
    std::shared_ptr<Ray> incidentRay;
    std::shared_ptr<SONAR_FILTER_BUFS> buffers;
    std::shared_ptr<FIRFilter<float, FILTER_LENGTH>> leftSonarFilter;
    std::shared_ptr<FIRFilter<float, FILTER_LENGTH>> centerSonarFilter;
    std::shared_ptr<FIRFilter<float, FILTER_LENGTH>> rightSonarFilter;
};


#endif //C_DETECTION_H
