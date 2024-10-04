#ifndef node_hpp
#define node_hpp

#include <glim_yhj/camera.hpp>

class Node
{
    private:
    public:
        Camera left_cam, right_cam;
        cv::Mat disparity16S, disparity8U, disparity32F;
        std::vector<PCL> pcl_vec;
        
        Node() = default;
        ~Node() = default;
        Node(const Camera& left_cam, const Camera& right_cam) : left_cam(left_cam), right_cam(right_cam)
        {

        }
};

#endif