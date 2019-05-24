//
// Created by Mason U'Ren on 2019-02-25.
//

#ifndef MULTIAGENTSLAM_NODE_H
#define MULTIAGENTSLAM_NODE_H

#include <iostream>
#include <array>

typedef struct {
    bool valid;
    uint16_t node_ptr;
} NODE_PTR;

class Node {
public:
    Node() :
        classifier(Classifier{}),
        featureSet(std::array<Feature, 3>
                {
                    (Feature{}),
                    (Feature{}),
                    (Feature{})
                }
        ),
        color(node_color::RED),
        location(NODE_PTR{.valid = false, .node_ptr = 0}),
        leftChild(NODE_PTR{.valid = false, .node_ptr = 0}),
        rightChild(NODE_PTR{.valid = false, .node_ptr = 0})
        {}
    ~Node() = default;

    bool operator==(const Node &node) const {
        return classifier.signature == node.classifier.signature;
    }

    Classifier classifier;
    std::array<Feature, 3> featureSet;
    node_color color;
    NODE_PTR location;
    NODE_PTR leftChild;
    NODE_PTR rightChild;

    void saveLocationPtr(const uint16_t &index) {
        if (!location.valid) {
            location.valid = true;
            location.node_ptr = index;
        }
    }
};

#endif //MULTIAGENTSLAM_NODE_H
