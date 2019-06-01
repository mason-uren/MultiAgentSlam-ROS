//
// Created by Mason U'Ren on 2019-02-20.
//

#ifndef MULTIAGENTSLAM_MOMENTS_H
#define MULTIAGENTSLAM_MOMENTS_H

#include <memory>
#include <iostream>
#include <sstream>
#include <array>

#include <templates/Filters/MeanFilter.h>
#include <templates/Filters/VarianceFilter.h>
#include <templates/Filters/CovarianceFilter.h>
#include <shared_structs/SharedMemoryStructs.h>

typedef struct {
    std::array<MeanFilter<float> *, ELEMENT_SIZE> means {{
        new MeanFilter<float>(),
        new MeanFilter<float>(),
        new MeanFilter<float>()
    }};
    std::array<VarianceFilter<float> *, ELEMENT_SIZE> variances {{
        new VarianceFilter<float>(),
        new VarianceFilter<float>(),
        new VarianceFilter<float>()
    }};
    std::array<CovarianceFilter<float> *, ELEMENT_SIZE> covariances {{
        new CovarianceFilter<float>(),
        new CovarianceFilter<float>(),
        new CovarianceFilter<float>()
    }};
} STATS;

class Moments {
public:
    static Moments *getInstance() {
        static Moments instance;
        return &instance;
    }

    Moments(Moments const &) = delete;
    void operator=(Moments const &) = delete;

    std::shared_ptr<STATS> motion;
    std::shared_ptr<STATS> measurement;

private:
    Moments() :
            motion(new STATS{}),
            measurement(new STATS{})
    {}
};




#endif //MULTIAGENTSLAM_MOMENTS_H
