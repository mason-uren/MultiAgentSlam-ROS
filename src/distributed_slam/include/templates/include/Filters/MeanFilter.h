//
// Created by Mason U'Ren on 2019-02-20.
//

#ifndef MULTIAGENTSLAM_MEANFILTER_H
#define MULTIAGENTSLAM_MEANFILTER_H

template <typename T>
class MeanFilter {
public:
    MeanFilter() :
        mean(0.0),
        population(1) {}

    T onlineAverage(const T &toAvg) {
        return mean += ((toAvg - mean) / population++);
    }

    T getFilteredValue() const {
        return mean;
    }

    void resetMean() {
        mean = 0.0;
        population = 1;
    }

private:
    T mean;
    unsigned long population;
};

#endif //MULTIAGENTSLAM_MEANFILTER_H
