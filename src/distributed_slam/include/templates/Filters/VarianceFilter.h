//
// Created by Mason U'Ren on 2019-02-20.
//

#ifndef MULTIAGENTSLAM_VARIANCEFILTER_H
#define MULTIAGENTSLAM_VARIANCEFILTER_H

template <typename T>
class VarianceFilter {
public:
    VarianceFilter() :
        variance(0),
        prevMean(0),
        sumOfSquaresOfDiff(0),
        population(1)
    {}

    ~VarianceFilter() = default;

    T onlineVariance(const T &input, const T &mean) {
        variance = sumOfSquaresOfDifferences(input, mean) / population;
        prevMean = mean;
        population++;
        return variance;
    }

    T getFilteredVariance() const {
        return variance;
    }

    void resetVariance() {
        variance = 0;
        prevMean = 0;
        sumOfSquaresOfDiff = 0;
        population = 1;
    }

private:
    T sumOfSquaresOfDifferences(T input, T mean) {
        return sumOfSquaresOfDiff += ((input - prevMean) * (input - mean));
    }

    T variance;
    T prevMean;
    float sumOfSquaresOfDiff;
    unsigned long population;
};

#endif //MULTIAGENTSLAM_VARIANCEFILTER_H
