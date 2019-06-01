//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_TRANSFORMATION_H
#define C_TRANSFORMATION_H

//#lib <SharedMemoryStructs.h>

#include <vector>
#include <shared_structs/SLAMConfigIn.h>
#include <templates/Filters/FIRFilter.h>

class Transformation {
public:
    Transformation() :
        x_translation(new FIRFilter<float, FILTER_LENGTH>(xBuf)),
        y_translation(new FIRFilter<float, FILTER_LENGTH>(yBuf)),
        orientation(new FIRFilter<float, FILTER_LENGTH>(orientBuf))
    {}
    virtual ~Transformation() = default;

    bool operator==(const Transformation &rhs) const {
        return x_translation->getValue() == rhs.x_translation->getValue() &&
            y_translation->getValue() == rhs.y_translation->getValue() &&
            orientation->getValue() == rhs.orientation->getValue();
    }

    FIRFilter<float, FILTER_LENGTH> *x_translation;
    FIRFilter<float, FILTER_LENGTH> *y_translation;
    FIRFilter<float, FILTER_LENGTH> *orientation;

private:
    float xBuf[FILTER_LENGTH]{};
    float yBuf[FILTER_LENGTH]{};
    float orientBuf[FILTER_LENGTH]{};
};

#endif //C_TRANSFORMATION_H