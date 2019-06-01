//
// Created by Mason U'Ren on 2019-02-13.
//
#ifndef C_FIRFILTER_H
#define C_FIRFILTER_H

#include <cstddef>
#include <cmath>
#include <cstring>
#include <iostream>

template <typename T, size_t N>
class FIRFilter {
public:
    FIRFilter(T (&buf)[N]) :
            head(0),
            bufferAtCapacity(false),
            accumTotal(0),
            buf(buf),
            bufferCapacity(N) {
        memset(buf, 0, sizeof(T) * bufferCapacity);
    }
    virtual ~FIRFilter() = default;;

    float filterValue(T sensorInput) {
        accumTotal += sensorInput;
        if (!bufferAtCapacity) {
            float filteredVal;
            buf[head++] = sensorInput;
            filteredVal = accumTotal / head;
            if (head == bufferCapacity) {
                bufferAtCapacity = true;
                head = 0;
            }
            return filteredVal;
        }
        else {
            accumTotal -= buf[head];
            buf[head++] = sensorInput;
            head %= bufferCapacity;
            return accumTotal / bufferCapacity;
        }
    }

    float getValue() {
        if (!bufferAtCapacity && head == 0) {
            return (float) nan("");
        }
        return bufferAtCapacity ? (accumTotal / bufferCapacity) : (accumTotal / head);
    }

    void printValues() {
        std::cout << "Filter history: " << std::endl;
        for (int val : buf) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }

    void clearBuffer() {
        head = 0;
        bufferAtCapacity = false;
        accumTotal = 0;
        memset(buf, 0, sizeof(T) * bufferAtCapacity);
    }


private:
    int head;
    bool bufferAtCapacity;
    float accumTotal;
    T *buf;
    long bufferCapacity;
};

#endif //C_FIRFILTER_H
