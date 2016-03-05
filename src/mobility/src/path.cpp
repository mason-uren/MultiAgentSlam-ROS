#include "path.hpp"

namespace csuci {
    PathNode::PathNode(double curr_x, double curr_y, double curr_theta, double x, double y)
    {
        next = NULL;
        prev = NULL;

        goal_pose.x = x;
        goal_pose.y = y;
        goal_pose.theta = atan2(y - curr_y, x - curr_x);

        turn_time = (goal_pose.theta - curr_theta) / DEFAULT_ANG_VELOCITY;
        move_time = hypot(goal_pose.x - curr_x, goal_pose.y - curr_y) / DEFAULT_VELOCITY;

        indx = 0;
    }

    PathNode::PathNode(PathNode* prev, double x, double y)
    {
        prev->next = this;
        this->prev = prev;

        next = NULL;

        goal_pose.x = x;
        goal_pose.y = y;
        goal_pose.theta = atan2(y - prev->goal_pose.y, x - prev->goal_pose.x);

        turn_time = (goal_pose.theta - prev->goal_pose.theta) / DEFAULT_ANG_VELOCITY;
        move_time = hypot(goal_pose.x - prev->goal_pose.x, goal_pose.y - prev->goal_pose.y) / DEFAULT_VELOCITY;

        indx = prev->indx + 1;
    }

    PathNode::PathNode(PathNode* prev, PathNode* next, double x, double y)
    {
        prev->next = this;
        next->prev = this;
        this->prev = prev;
        this->next = next;

        goal_pose.x = x;
        goal_pose.y = y;
        goal_pose.theta = atan2(y - prev->goal_pose.y, x - prev->goal_pose.x);

        turn_time = (goal_pose.theta - prev->goal_pose.theta) / DEFAULT_ANG_VELOCITY;
        move_time = hypot(goal_pose.x - prev->goal_pose.x, goal_pose.y - prev->goal_pose.y) / DEFAULT_VELOCITY;

        indx = prev->indx + 1;

        double next_turn_time = (next->goal_pose.theta - goal_pose.theta) / DEFAULT_ANG_VELOCITY;
        double next_move_time = hypot(next->goal_pose.x - goal_pose.x, next->goal_pose.y - goal_pose.y) / DEFAULT_VELOCITY;

        next->turn_time = next_turn_time;
        next->move_time = next_move_time;

        PathNode* node = next;
        if(node && node->next->indx == indx) {
            while(node && node->next) {
                node->indx = node->indx + 1;
                node = node->next;
            }
        }

    }

    PathNode::PathNode(const PathNode& pn)
    {
        prev = pn.prev;
        next = pn.next;

        goal_pose.x = pn.goal_pose.x;
        goal_pose.y = pn.goal_pose.y;
        goal_pose.theta = pn.goal_pose.theta;

        move_time = pn.move_time;
        turn_time = pn.turn_time;
        indx = pn.indx;
    }

    PathNode::~PathNode()
    {

    }

    PathNode& PathNode::operator=(const PathNode& pn)
    {
        prev = pn.prev;
        next = pn.next;

        goal_pose.x = pn.goal_pose.x;
        goal_pose.y = pn.goal_pose.y;
        goal_pose.theta = pn.goal_pose.theta;

        move_time = pn.move_time;
        turn_time = pn.turn_time;
        indx = pn.indx;

        return *this;
    }

    PathNode* PathNode::Next()
    {
        return next;
    }

    PathNode* PathNode::Prev()
    {
        return prev;
    }

    const geometry_msgs::Pose2D& PathNode::Goal() const
    {
        return goal_pose;
    }

    double PathNode::TotalTravelTime() const
    {
        return turn_time + move_time;
    }

    double PathNode::TurnTime() const
    {
        return turn_time;
    }

    double PathNode::MoveTime() const
    {
        return move_time;
    }

    int PathNode::Index() const
    {
        return indx;
    }

    void PathNode::RemoveSelf()
    {
        if(prev) {
            prev->next = this->next;
        }

        if(next) {
            next->prev = this->prev;

            PathNode* node = next;

            while(node) {
                node->indx -= 1;
                node = node->next;
            }
        }
    }


    Path::Path()
    {
        head = NULL;
        tail = NULL;
        size = 0;
    }

    Path::~Path()
    {
        PathNode* node = head;

        while(node) {
            PathNode* next = node->next;
            delete node;
            node = next;
        }
    }

    void Path::Add(double curr_x, double curr_y, double curr_theta, double x, double y)
    {
        Insert(size, curr_x, curr_y, curr_theta, x, y);
    }

    void Path::Insert(size_t idx, double curr_x, double curr_y, double curr_theta, double x, double y)
    {
        if(head == NULL) {
            head = new PathNode(curr_x, curr_y, curr_theta, x, y);
            tail = head;
        } else {
            PathNode* node = head;
            while(node->next && node->indx < idx) {
                node = node->next;
            }

            if(node == head) {
                if(node->indx >= idx) {
                    node->prev = new PathNode(curr_x, curr_y, curr_theta, x, y);
                    node->prev->next = node;
                    head = node->prev;
                } else {
                    head->next = new PathNode(head, x, y);
                    tail = head->next;
                }
            } else if(node == tail) {
                node->next = new PathNode(tail, x, y);
                tail = node->next;
            } else {
                PathNode* pn = new PathNode(node->prev, node, x, y);
            }
        }

        size++;
    }

    PathNode* Path::Get(size_t idx)
    {
        if(head) {
            PathNode* node = head;
            size_t i = 0;

            while(node && i < idx) {
                i = node->indx;
                node = node->next;
            }

            if(node->indx == i) {
                return node;
            } else {
                return (PathNode*) NULL;
            }
        } else {
            return (PathNode*) NULL;
        }
    }

    void Path::Remove(size_t idx)
    {
        if(head) {
            PathNode* node = head;
            size_t i = 0;

            while(node && i != idx) {
                i = node->indx;
                node = node->next;
            }

            if(node == NULL) {
                return;
            }

            if(node == head) {
                head = node->next;
            }

            if(node == tail) {
                tail = node->prev;
            }

            node->RemoveSelf();
            delete node;
            size--;
        }
    }

    void Path::RemoveRange(size_t start, size_t end)
    {
        if(head) {
            PathNode* node = head;
            size_t i = 0;

            if(end < start) {
                size_t tmp = end;
                end = start;
                start = tmp;
            }

            while(node && i < start) {
                i = node->indx;
                node = node->next;
            }

            if(node == NULL) {
                return;
            }

            PathNode* next_node = NULL;

            while(node && i <= end) {
                next_node = node->next;

                if(node == head) {
                    head = node->next;
                }

                if(node == tail) {
                    tail = node->prev;
                }

                node->RemoveSelf();
                delete node;
                node = next_node;
                size--;

                if(node) {
                    i = node->indx;
                }
            }

        }
    }

    void Path::RemoveAfter(size_t idx)
    {
        RemoveRange(idx, size-1);
    }

    size_t Path::Size() const
    {
        return size;
    }
}
