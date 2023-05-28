#pragma once
#include <vector>
#include <algorithm>
#include "obstacle_detector/utilities/point.h"

namespace obstacle_detector
{
    typedef std::vector<Point>::iterator PointIterator;
    struct Node
    {
        Point point;
        Node* left;
        Node* right;
        // Add any other required attributes
    };

    class KDTree
    {
    public:
        KDTree() : root(nullptr) {}

        void build(const std::vector<Point>& points)
        {
            // Create a copy of the points vector
            std::vector<Point> sortedPoints = points;

            // Build the kd-tree recursively
            root = buildRecursive(sortedPoints.begin(), sortedPoints.end(), 0);
        }
        void inOrderTraversal(Node* node, std::vector<Point>& sortedPoints)
        {
            if (node == nullptr)
                return;

            // Recurse on the left subtree
            inOrderTraversal(node->left, sortedPoints);

            // Insert the current node's point into the list
            sortedPoints.push_back(node->point);

            // Recurse on the right subtree
            inOrderTraversal(node->right, sortedPoints);
        }
        void clear()
        {
            clearRecursive(root);
            root = nullptr;
        }
        Node* root;

    private:
        struct ComparePoints
        {
            int axis;

            ComparePoints(int axis) : axis(axis) {}

            bool operator()(const Point& p1, const Point& p2) const
            {
                if (axis == 0)
                    return p1.x < p2.x;
                else if (axis == 1)
                    return p1.y < p2.y;
            }
        };

        Node* buildRecursive(PointIterator begin, PointIterator end, int depth)
        {
            if (begin == end)
                return nullptr;

            int axis = depth % 2; // Assuming 3D points

            std::sort(begin, end, ComparePoints(axis));
            PointIterator mid = begin + (end - begin) / 2;

            Node* node = new Node;
            node->point = *mid;
            node->left = buildRecursive(begin, mid, depth + 1);
            node->right = buildRecursive(mid + 1, end, depth + 1);

            return node;
        }

        void clearRecursive(Node* node)
        {
            if (node == nullptr)
                return;

            clearRecursive(node->left);
            clearRecursive(node->right);
            delete node;
        }
    };

    class PointSet
    {
    public:
        PointSet() : num_points(0), is_visible(false) {}

        PointIterator begin, end; // The iterators point to the vector of points existing somewhere else
        int num_points;
        bool is_visible; // The point set is not occluded by any other point set
    };
} // namespace obstacle_detector
