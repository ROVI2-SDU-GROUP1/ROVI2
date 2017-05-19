#include<limits>
#include<algorithm>
#include<TreePlotter.hpp>
#include <vector>
cv::Point2f to_image_coordinates(double x, double x_min, double x_max, uint32_t width, double y, double y_min, double y_max, uint32_t height)
{
    cv::Point2f p;
    p.x = (x - x_min) * (width / (x_max - x_min));
    p.y = (y - y_min) * (height / (y_max - y_min));
    p.y = height - p.y; //Flip y axis
    return p;
}

void plotNode(cv::Point2f node, cv::Mat &outImage)
{
    return;
}

void plotEdge(cv::Point2f n1, cv::Point2f n2, cv::Mat &outImage, cv::Scalar col = cv::Scalar(0, 0, 0), int size = 1)
{
    cv::line(outImage, n1, n2, col, size);
}

void plotTree_rec(   const RT_Node *node, cv::Mat &outImage, uint32_t width, uint32_t height,
                double x_max, double x_min, double y_max, double y_min,
                uint8_t x_axis_dim, uint8_t y_axis_dim)
{
    cv::Point2f node_coord = to_image_coordinates(  node->getValue()[x_axis_dim], x_min, x_max, width,
                                                    node->getValue()[y_axis_dim], y_min, y_max, height);

    for(auto child : node->get_childs())
    {
        cv::Point2f child_coord = to_image_coordinates(  child->getValue()[x_axis_dim], x_min, x_max, width,
                                                         child->getValue()[y_axis_dim], y_min, y_max, height);
        plotEdge(node_coord, child_coord, outImage);
        plotTree_rec(child, outImage, width, height, x_max, x_min, y_max, y_min, x_axis_dim, y_axis_dim);
    }
    plotNode(node_coord, outImage);
}

void find_max_min_recursive(const RT_Node *node, double &x_max, double &x_min, double &y_max, double &y_min, uint8_t x_axis_dim, uint8_t y_axis_dim)
{
    for(auto child : node->get_childs())
    {
        find_max_min_recursive(child, x_max, x_min, y_max, y_min, x_axis_dim, y_axis_dim);
    }
    y_min = std::min(node->getValue()[y_axis_dim], y_min);
    x_min = std::min(node->getValue()[x_axis_dim], x_min);
    y_max = std::max(node->getValue()[y_axis_dim], y_max);
    x_max = std::max(node->getValue()[x_axis_dim], x_max);
}

void plotTree(const RT_Node *treeRoot, cv::Mat &outImage, uint8_t x_axis_dim, uint8_t y_axis_dim, std::vector<RT_Node *> goal_path)
{
    outImage = cv::Scalar(255, 255, 255);
    //find x and y limits
    uint32_t width = outImage.cols;
    uint32_t height = outImage.rows;
    double y_min = std::numeric_limits<double>::max();
    double x_min = y_min;
    double y_max = std::numeric_limits<double>::min();
    double x_max = y_max;

    find_max_min_recursive(treeRoot, x_max, x_min, y_max, y_min, x_axis_dim, y_axis_dim);

    plotTree_rec(treeRoot, outImage, width, height, x_max, x_min, y_max, y_min, x_axis_dim, y_axis_dim);
    //Plot the path
    if(goal_path.size() == 0) return;
    for(size_t i = 0; i < goal_path.size() - 1; i++)
    {
        cv::Point2f n1 = to_image_coordinates(  goal_path[i]->getValue()[x_axis_dim], x_min, x_max, width,
                                                goal_path[i]->getValue()[y_axis_dim], y_min, y_max, height);
        cv::Point2f n2 = to_image_coordinates(  goal_path[i + 1]->getValue()[x_axis_dim], x_min, x_max, width,
                                                goal_path[i + 1]->getValue()[y_axis_dim], y_min, y_max, height);
        plotEdge(n1, n2, outImage, cv::Scalar(255, 0, 0), 3);
    }
}
