//
// Created by Mason U'Ren on 2019-02-25.
//

#include "RedBlackTree.h"

using boost::math::normal;

static normal normalDist = normal();

bool RedBlackTree::findMLClassifier(const Classifier &classifier, uint16_t &index) {
    Node *node;
    float prob{0};
    auto currNodePtr{NODE_PTR()};
    this->setChild(&currNodePtr, this->root);

    while (true) {
        if (!currNodePtr.valid) {
            return prob > this->minProbability;
        }

        node = &this->tree[currNodePtr.node_ptr];
        float qArea = this->areaLikelihood(node->classifier.area - classifier.area);
        float qOrient = this->orientationLikelihood(node->classifier.orientation - classifier.orientation);
        float avg = Equations::getInstance()->straightAvg({qArea, qOrient});
        float likelihood = Equations::getInstance()->normalizeValue(avg, 0, 1);

        if (likelihood > std::max(prob, this->minProbability)) {
            prob = likelihood;
            index = currNodePtr.node_ptr;
        }

        // Right child
        if (classifier.signature > node->classifier.signature) {
            currNodePtr = node->rightChild;
        }
        // Left child
        else if (classifier.signature < node->classifier.signature) {
            currNodePtr = node->leftChild;
        }
        // Identical signatures
        else {
            index = currNodePtr.node_ptr;
            return true;
        }
    }
}

void RedBlackTree::addToTree(
        const Classifier &classifier,
        const std::array<Feature, 3> &features) {
    auto node{&this->tree[this->availableIndexPtr]};
    node->classifier = classifier;
    node->featureSet = features;
    node->saveLocationPtr(this->availableIndexPtr);

    this->balanaceTree(node->classifier.signature);
    this->incrimentPtr();
}

void RedBlackTree::resetTree() {
    this->availableIndexPtr = 0;
    this->changeRootPtr(this->availableIndexPtr);
    this->tree = this->cleanTree;
}

void RedBlackTree::getFeaturesFromNode(
        std::array<Feature, 3> &featuresToPopulate,
        const uint16_t &nodeIndexPtr) {
    featuresToPopulate = this->tree[nodeIndexPtr].featureSet;
}

float RedBlackTree::areaLikelihood(const float &area) {
   return (float) pdf(normalDist, area);
}

float RedBlackTree::orientationLikelihood(const float &orient) {
    return (float) pdf(normalDist, orient);
}

uint16_t RedBlackTree::singleRotation(const uint16_t &nodeIndex, const dir &direction) {
    uint16_t tempPtr;
    auto otherPtr{NODE_PTR()};
    auto *node{&this->tree[nodeIndex]};
    Node *other;

    if (!direction) {
        tempPtr = node->rightChild.node_ptr;
        node->rightChild = (other = &this->tree[tempPtr]) ? other->leftChild : otherPtr;
        if (other) {
            this->setChild(&other->leftChild, nodeIndex);
            other->color = node_color::BLACK;
        }
    }
    else {
        tempPtr = node->leftChild.node_ptr;
        node->leftChild = (other = &this->tree[tempPtr]) ? other->rightChild : otherPtr;
        if (other) {
            this->setChild(&other->rightChild, nodeIndex);
            other->color = node_color::BLACK;
        }
    }
    node->color = node_color::RED;

    return tempPtr;
}

uint16_t RedBlackTree::doubleRotation(const uint16_t &nodeIndex, const dir &direction) {
    auto ref{this->tree[nodeIndex]};
    uint16_t tempPtr;

    bool hasLeaf = (!direction) ?
            this->assignChildTo(ref.rightChild, &tempPtr) :
            this->assignChildTo(ref.leftChild, &tempPtr);

    this->rotateNodes(
            this->changeDir(direction), ref,
            tempPtr = this->singleRotation(
                    ((hasLeaf) ? tempPtr : -1), this->changeDir(direction)
            )
    );

    return this->singleRotation(nodeIndex, direction);
}

bool RedBlackTree::isRed(const uint16_t *nodeIndex) {
    if (nodeIndex) {
        auto ref{this->tree[*nodeIndex]};
        return ref.color == node_color::RED;
    }
    return false;
}

bool RedBlackTree::isRed(const Node *node) {
    return node && node->color == node_color::RED;
}

void RedBlackTree::incrimentPtr() {
    this->availableIndexPtr++;
}

void RedBlackTree::changeRootPtr(const uint16_t &index) {
    this->root = index;
}

void RedBlackTree::changeNodeColor(const uint16_t &nodeIndex, const node_color *desiredColor = nullptr) {
    if (Node *node = &this->tree[nodeIndex]) {
        node->color = (desiredColor) ? *desiredColor : (((int) node->color) ? node_color::BLACK : node_color::RED);
    }
}

void RedBlackTree::balanaceTree(const float &signature) {
    static bool isFirstNode = false;
    if (!isFirstNode) {
        this->root = this->availableIndexPtr;
        isFirstNode = true;
    }
    else {
        // False tree root
        auto head{Node()};
        Node *grandparent = nullptr, *temp = nullptr, *parent = nullptr, *iterator = nullptr;
        dir direction = dir::LEFT;
        dir lastDir = dir::LEFT;

        temp = &head;
        this->setChild(&temp->rightChild, this->root);
        iterator = &this->tree[this->root];

        while (true) {
            if (!iterator) {
                // Connect added node to tree
                iterator = &this->tree[this->availableIndexPtr];
                if (direction) {
                    this->setChild(&parent->rightChild, this->availableIndexPtr);
                } else {
                    this->setChild(&parent->leftChild, this->availableIndexPtr);
                }

            }
            // Color flip
            else if (this->isRed(&(iterator->leftChild.node_ptr)) && this->isRed(&(iterator->rightChild.node_ptr))) {
                iterator->color = node_color::RED;
                node_color color = node_color::BLACK;
                this->changeNodeColor(iterator->leftChild.node_ptr, &color);
                this->changeNodeColor(iterator->rightChild.node_ptr, &color);
            }

            // Fix red violation
            if (this->isRed(iterator) && this->isRed(parent)) {
                auto node{this->tree[temp->rightChild.node_ptr]};
                dir tempDir = (node == *grandparent) ? dir::RIGHT : dir::LEFT;

                uint16_t childPtr;
                bool hasLeaf = (lastDir) ?
                    this->assignChildTo(parent->rightChild, &childPtr) :
                    this->assignChildTo(parent->leftChild, &childPtr);

                if (iterator == (hasLeaf ? &this->tree[childPtr] : nullptr)) {
                    this->rotateNodes(
                            tempDir, *temp,
                            this->singleRotation(grandparent->location.node_ptr, this->changeDir(lastDir))
                    );
                }
                else {
                    this->rotateNodes(
                            tempDir, *temp,
                            this->doubleRotation(grandparent->location.node_ptr, this->changeDir(lastDir))
                    );
                }
            }

            // Stop if added node was found
            if (iterator->classifier.signature == signature) {
                break;
            }

            lastDir = direction;
            direction = (iterator->classifier.signature < signature) ? dir::RIGHT : dir::LEFT;

            // Update Helpers
            if (grandparent) {
                temp = grandparent;
            }

            grandparent = parent;
            parent = iterator;


            bool hasRight = iterator->rightChild.valid;
            bool hasLeft = iterator->leftChild.valid;
            iterator = (direction) ?
                       ((hasRight) ? &this->tree[iterator->rightChild.node_ptr] : nullptr) :
                       ((hasLeft) ? &this->tree[iterator->leftChild.node_ptr] : nullptr);
        }

        this->changeRootPtr(head.rightChild.node_ptr);
    }
    this->tree[this->root].color = node_color::BLACK;
}

dir RedBlackTree::changeDir(const dir &current) {
    return current ? dir::LEFT : dir::RIGHT;
}

void RedBlackTree::rotateNodes(const dir &ofRotation, Node &parent, const uint16_t &index) {
    (ofRotation) ?
        this->setChild(&(parent.rightChild), index) :
        this->setChild(&(parent.leftChild), index);
}

void RedBlackTree::setChild(NODE_PTR *ptr, const uint16_t &newValue) {
    if (ptr) {
        ptr->valid = true;
        ptr->node_ptr = newValue;
    }
}

bool RedBlackTree::assignChildTo(const NODE_PTR &node_ptr, uint16_t *otherPtr) {
    *otherPtr = node_ptr.node_ptr;
    return node_ptr.valid;
}

// For testing purposes only!
void RedBlackTree::printTree(NODE_PTR *root, int level) {
    if (root->valid) {
        auto node{this->tree[root->node_ptr]};
        std::cout << "Signature : " << node.classifier.signature << " (" << ((int) node.color) <<
            ") Level -> (" << level << ")" << std::endl;
        printTree(&node.leftChild, ++level);
        level--;
        printTree(&node.rightChild, ++level);
        level--;
    }
}
