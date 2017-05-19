#include<RT_RRT_Tree.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
void plotTree(const RT_Node *treeRoot, cv::Mat &outImage, uint8_t x_axis_dim, uint8_t y_axis_dim, std::vector<RT_Node *> goal_path);
