#include<limits>
#include<algorithms>
void plotTree(const <RT_RRT_Tree<rw::math::Q> &tree cv::Mat &outImage, uint8_t x_axis_dim, uint8_t y_axis_dim, uint32_t width = 1280, uint32_t height = 800)
{
    //find x and y limits
    double y_min = std::numeric_limits<double>::max();
    double x_min = y_min;
    double y_max = std::numeric_limits<double>::min();
    double x_max = y_max;
    for(auto node : tree)
    {
        y_min = std::min(node->getValue()[x_axis_dim], y_min);
        x_min = std::min(node->getValue()[x_axis_dim], x_min);
        y_max = std::max(node->getValue()[x_axis_dim], y_max);
        x_max = std::max(node->getValue()[x_axis_dim], x_max);
    }
    //Plot nodes and edges
    for(auto node : tree)
    {
        cv::Point2f node_coord = to_image_coordinates(  node->getValue()[x_axis_dim], x_min, x_max, width,
                                                        node->getValue()[y_axis_dim], y_min, y_max, height);
        for(auto child : node->get_childs())
        {
            cv::Point2f child_coord = to_image_coordinates(  child->getValue()[x_axis_dim], x_min, x_max, width,
                                                             child->getValue()[y_axis_dim], y_min, y_max, height);
            plotEdge(node_coord, child_coord);
        }
        plotNode(node_coord);
    }


}

cv::Point2f to_image_coordinates(double x, double x_min, double x_max, uint32_t width, double y, double y_min, double y_max, uint32_t height)
{
    cv::Point2f p;
    p.x = (width / (x_max - x_min) ) * (x - x_min);
    p.y = (height / (y_max - y_min) ) * (y - y_min);
    return p;
}

void plotNode(cv::Point2f node, cv::Mat &outImage)
{
    //Do some plotting here
}

void plotEdge(cv::Point2f n1, cv::Point2f n2, cv::Mat &outImage)
{
    //Do some more awesome plotting here!
}
