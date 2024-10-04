#ifndef utils_hpp
#define utils_hpp

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/opencv.hpp>

#include "glim_yhj/data_type.hpp"
#include "glim_yhj/node.hpp"

class Utils
{
    private:
        cv::Ptr<cv::StereoSGBM> sgbm;
        int minDisparity = 0;
        int numDisparities = 16 * 5;
        int blockSize = 7;
        int P1 = 8 * blockSize * blockSize;
        int P2 = 32 * blockSize * blockSize;
    public:
        Utils()
        {
            sgbm = cv::StereoSGBM::create(minDisparity, numDisparities, blockSize, P1, P2);
        }
        ~Utils() = default;
        void getDisparityImage(Node& node)
        {
            sgbm->compute(node.left_cam.gray_image, node.right_cam.gray_image, node.disparity16S);
            
            double minVal, maxVal;
            cv::minMaxLoc(node.disparity16S, &minVal, &maxVal);
            node.disparity16S.convertTo(node.disparity8U, CV_8UC1, 255 / (maxVal - minVal));
            node.disparity16S.convertTo(node.disparity32F, CV_32F, 1.f / 16.f);
        }

        sensor_msgs::PointCloud2 convert_PointCloud2(const std::vector<PCL>& data, const std::string& frame_name)
        {
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            cloud.width = data.size();
            cloud.height = 1;
            cloud.is_dense = false;
            cloud.points.resize(cloud.width * cloud.height);
            
            for(size_t i=0; i<data.size(); i++) 
            {
                cloud.points[i].x = data[i].pts(0);
                cloud.points[i].y = data[i].pts(1);
                cloud.points[i].z = data[i].pts(2);
                cloud.points[i].r = data[i].clr(0);
                cloud.points[i].g = data[i].clr(1);
                cloud.points[i].b = data[i].clr(2);
            }

            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(cloud, output);
            output.header.frame_id = frame_name;
            return output;
        }

        void PointClouds(Node& node)
        {
            node.pcl_vec.clear();

            int rows = node.disparity32F.rows;
            int cols = node.disparity32F.cols;

            double fx = node.left_cam.intrinsic_mat.at<double>(0,0);
            double fy = node.left_cam.intrinsic_mat.at<double>(1,1);
            double cx = node.left_cam.intrinsic_mat.at<double>(0,2);
            double cy = node.left_cam.intrinsic_mat.at<double>(1,2);
            double bl = -0.5333;
            
            PCL pcl;
            double e, f, disp, depth;
            for(int r = 0; r < rows; r++)
            {
                for(int c = 0; c < cols; c++) 
                {
                    e = (c - cx) / fx;
                    f = (r - cy) / fy;

                    disp  = node.disparity32F.at<float>(r,c);
                    if(disp <= 0.0){continue;}
                    
                    depth = fx * bl / disp;
                    
                    pcl.pts[0] = e * depth; // camera coordinate x
                    pcl.pts[1] = f * depth; // camera coordinate y
                    pcl.pts[2] = depth;     // camera coordinate z

                    node.pcl_vec.push_back(pcl);
                }
            }
        }
};

#endif