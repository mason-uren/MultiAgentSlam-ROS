//
// Created by Mason U'Ren on 2019-02-25.
//

#ifndef MULTIAGENTSLAM_REDBLACKTREE_H
#define MULTIAGENTSLAM_REDBLACKTREE_H

#include <vector>
#include <algorithm>
#include <limits>
#include <cstddef>
#include <functional>

#include <include/SharedMemoryStructs.h>
#include <include/Node/Node.h>

#include <boost/math/distributions/normal.hpp>

#include "../../Utilities/Equations/Equations.h"

#define PDF_MAX 0.4 // Roughly

enum dir {
    LEFT = 0,
    RIGHT
};

class RedBlackTree {
public:
    explicit RedBlackTree(LOCAL_MAP_CONFIG *localMapConfig) :
        availableIndexPtr(0),
        root(0),
        tree(std::vector<Node>(localMapConfig->maxFeatures)),
        cleanTree(std::vector<Node>(localMapConfig->maxFeatures)),
        minProbability((float) (localMapConfig->featureSetML * PDF_MAX)) {}
    ~RedBlackTree() = default;

    bool findMLClassifier(const Classifier &classifier, uint16_t &index);
    void addToTree(const Classifier &classifier, const std::array<Feature, FEATURE_LIMIT> &features);
    void resetTree();
    void getFeaturesFromNode(std::array<Feature, FEATURE_LIMIT> &featuresToPopulate,
            const uint16_t &nodeIndexPtr);

    // For testing purposes only!
    void printTree(NODE_PTR *root, int  level);
    uint16_t *getRoot() {
        return &(root);
    }

private:
    float areaLikelihood(const float &area);
    float orientationLikelihood(const float &orient);
    uint16_t singleRotation(const uint16_t &nodeIndex, const dir &direction);
    uint16_t doubleRotation(const uint16_t &nodeIndex, const dir &direction);
    bool isRed(const uint16_t *nodeIndex);
    bool isRed(const Node *node);
    void incrimentPtr();
    void changeRootPtr(const uint16_t &index);
    void changeNodeColor(const uint16_t &nodeIndex, const node_color *desiredColor);
    void balanaceTree(const float &signature);
    dir changeDir(const dir &current);
    void rotateNodes(const dir &ofRotation, Node &parent, const uint16_t &index);
    void setChild(NODE_PTR *ptr, const uint16_t &newValue);
    bool assignChildTo(const NODE_PTR &node_ptr, uint16_t *otherPtr);

    uint16_t availableIndexPtr;
    uint16_t root;
    std::vector<Node> tree;
    std::vector<Node> cleanTree;
    float minProbability;
};


#endif //MULTIAGENTSLAM_REDBLACKTREE_H
