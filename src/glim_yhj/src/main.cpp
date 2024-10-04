#include "glim_yhj/glim_yhj.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "glim_yhj");
    ros::NodeHandle nh;
    ros::Publisher pub_points = nh.advertise<sensor_msgs::PointCloud2>("points", 1);

    int num_images;
    cv::Mat intrinsic_mat;
    std::string file_path = "/root/data_odometry_color/dataset/sequences/00";
    DatasetLoader(file_path, num_images, intrinsic_mat);
    Utils UT;
    
    ros::Rate rate(30);
    for(int i = 0; i < num_images && ros::ok(); i++)
    {
        Camera left_cam(file_path + cv::format("/image_2/%06d.png", i), intrinsic_mat);
        Camera right_cam(file_path + cv::format("/image_3/%06d.png", i), intrinsic_mat);
        
        Node node(left_cam, right_cam);
        UT.getDisparityImage(node);
        UT.PointClouds(node);
        pub_points.publish(UT.convert_PointCloud2(node.pcl_vec, "map"));

        cv::imshow("left_cam.image", left_cam.image);
        cv::imshow("disparity8U", node.disparity8U);
        char k = cv::waitKey(1);
        if(k == 'q')
            break;

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}