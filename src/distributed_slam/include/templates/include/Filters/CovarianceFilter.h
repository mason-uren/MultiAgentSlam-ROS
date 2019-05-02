//
// Created by Mason U'Ren on 2019-03-21.
//

#ifndef MULTIAGENTSLAM_COVARIANCEFILTER_H
#define MULTIAGENTSLAM_COVARIANCEFILTER_H

template <typename T>
class CovarianceFilter {
public:
    CovarianceFilter() :
        covariance(0),
        prevXBar(0),
        prevYBar(0),
        population(1)
    {}
    ~CovarianceFilter() = default;

    T onlineCovariance(const T &x, const T &y, const T &xBar, const T &yBar) {
        T temp = (population - 1) / (float) population;
        covariance = (((covariance * (population - 1) )) +  (temp * (x - prevXBar) * (y - prevYBar))) / population;
        prevXBar = xBar;
        prevYBar = yBar;
        population++;
        return covariance;
    }

    T getFilteredCovariance() {
        return covariance;
    }

    void resetCovariance() {
        covariance = 0;
        prevXBar = 0;
        prevYBar = 0;
        population = 1;
    }

private:
    T covariance;
    T prevXBar;
    T prevYBar;
    unsigned long population;
};

#endif //MULTIAGENTSLAM_COVARIANCEFILTER_H
