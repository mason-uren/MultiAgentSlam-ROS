//
// Created by Mason U'Ren on 2019-02-25.
//

#ifndef MULTIAGENTSLAM_NODE_H
#define MULTIAGENTSLAM_NODE_H

#include <iostream>
#include <array>
#include <boost/math/distributions/normal.hpp>

using namespace boost::math;

typedef struct {
    bool valid;
    uint16_t node_ptr;
} NODE_PTR;

class Node {
public:
    Node() :
        classifier(CLASSIFIER{.area = 0, .orientation = 0, .signature = 0}),
        featureSet(std::array<FEATURE, 3>
                {
                    (FEATURE{}),
                    (FEATURE{}),
                    (FEATURE{})
                }
        ),
        color(node_color::RED),
        location(NODE_PTR{.valid = false, .node_ptr = 0}),
        leftChild(NODE_PTR{.valid = false, .node_ptr = 0}),
        rightChild(NODE_PTR{.valid = false, .node_ptr = 0})
        {}
    ~Node() = default;

    bool operator == (const Node &node) const {
        return classifier.signature == node.classifier.signature;
    }

    CLASSIFIER classifier;
    std::array<FEATURE, 3> featureSet;
    node_color color;
    NODE_PTR location;
    NODE_PTR leftChild;
    NODE_PTR rightChild;

    static const std::shared_ptr<normal> distribution;

    void saveLocationPtr(const uint16_t &index) {
        if (!location.valid) {
            location.valid = true;
            location.node_ptr = index;
        }
    }
};

#endif //MULTIAGENTSLAM_NODE_H
