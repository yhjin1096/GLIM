#ifndef __camera_hpp__
#define __camera_hpp__

#include <iostream>
#include <opencv2/opencv.hpp>

#include <glim_yhj/data_type.hpp>

class Camera
{
    private:
    public:
        cv::Mat intrinsic_mat;
        cv::Mat image, gray_image;

        Pose pose;

        Camera(const std::string& file_path, const cv::Mat& intrinsic_mat) : intrinsic_mat(intrinsic_mat)
        {
            image = cv::imread(file_path);
            cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
        };
        ~Camera(){};
};

#endif