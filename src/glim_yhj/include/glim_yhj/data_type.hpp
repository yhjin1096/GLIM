#ifndef data_type_hpp
#define data_type_hpp

#include <opencv2/opencv.hpp>

struct PCL 
{
  cv::Vec3f pts;
  cv::Vec3b clr;
  PCL()
  {
    pts[0]=0.0; pts[1]=0.0; pts[2]=0.0;
    clr[0]=255; clr[1]=255; clr[2]=255;
  }
};

struct Pose
{
    cv::Mat c2w_R, c2w_t;
    cv::Mat cam_to_world;

    cv::Mat w2c_R, w2c_t;
    cv::Mat world_to_cam;

    Pose()
    {
      cam_to_world = cv::Mat::eye(4, 4, CV_64F);
      world_to_cam = cv::Mat::eye(4, 4, CV_64F);
      c2w_R = cv::Mat::eye(3, 3, CV_64F);
      w2c_R = cv::Mat::eye(3, 3, CV_64F);
      c2w_t = cv::Mat::zeros(3, 1, CV_64F);
      w2c_t = cv::Mat::zeros(3, 1, CV_64F);
      // c2w_t.at<double>(2, 0) = 1;
      // w2c_t.at<double>(2, 0) = 1;
    }

    void setPose(const cv::Mat& R, const cv::Mat& t)
    {
      c2w_R = R.clone();
      c2w_t = t.clone();

      cv::Mat rigid_body_tf;
      cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
      cv::hconcat(c2w_R, c2w_t, rigid_body_tf);
      cv::vconcat(rigid_body_tf, addup, cam_to_world);

      world_to_cam = cam_to_world.inv();
      w2c_R = world_to_cam(cv::Rect(0,0,3,3));
      w2c_t = world_to_cam(cv::Rect(3,0,1,3));
    }
};

#endif