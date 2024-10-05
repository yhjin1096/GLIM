#ifndef dataset_loader_hpp
#define dataset_loader_hpp

#include <string>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>

#include <glim_yhj/data_type.hpp>

class DatasetLoader
{
    private:
    public:
        DatasetLoader(const std::string& dataset_path, int& num_images, cv::Mat& intrinsic_mat, std::vector<double>& time_seq)
        {
            num_images = countImages(dataset_path + "/image_2");
            intrinsic_mat = readIntrinsicMat(dataset_path + "/calib.txt").clone();
            time_seq = getTimeSequence(dataset_path + "/times.txt");
        }
        ~DatasetLoader() = default;

        int countImages(const std::string &path);
        std::vector<Pose> readGTPose(const std::string& path);
        cv::Mat readIntrinsicMat(const std::string& path);
        std::vector<double> getTimeSequence(const std::string& path);
};

const std::string imageExtensions[] = {".jpg", ".jpeg", ".png", ".gif", ".bmp"};
int DatasetLoader::countImages(const std::string &path)
{
    int num_images = 0;
    try
    {
        // 지정된 폴더 내의 모든 파일에 대해 반복
        for (const auto &entry : boost::filesystem::directory_iterator(path))
        {
            // 디렉토리인 경우 건너뛰기
            if (boost::filesystem::is_directory(entry.path()))
                continue;

            // 이미지 파일인 경우 개수 증가
            for (const std::string &ext : imageExtensions)
            {
                if (entry.path().extension() == ext)
                    num_images++;
            }
        }

        ROS_INFO("Number of image files in the folder: %d", num_images);
        
        return num_images;
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Error: " << ex.what() << std::endl;
        exit(-1);
    }
}

std::vector<Pose> DatasetLoader::readGTPose(const std::string& path)
{
    std::ifstream file(path);
    std::string line, word;
    
    std::vector<Pose> gt_poses;
    
    if(file.is_open())
    {
        while(getline(file, line))
        {
            int i = 0, j = 0;

            std::stringstream ss(line);
            Pose gt_pose;
            cv::Mat gt_mat = cv::Mat::eye(4, 4, CV_64F);
            
            while(getline(ss, word, ' '))
            {
                gt_mat.at<double>(j,i) = std::stod(word);

                i++;
                if(i==4)
                {
                    i=0;
                    j++;
                }
            }
            gt_pose.cam_to_world = gt_mat.clone();
            gt_pose.c2w_R = gt_pose.cam_to_world(cv::Rect(0,0,3,3));
            gt_pose.c2w_t = gt_pose.cam_to_world(cv::Rect(3,0,1,3));
            gt_pose.world_to_cam = gt_pose.cam_to_world.inv();
            gt_pose.w2c_R = gt_pose.world_to_cam(cv::Rect(0,0,3,3));
            gt_pose.w2c_t = gt_pose.world_to_cam(cv::Rect(3,0,1,3));

            gt_poses.push_back(gt_pose);
        }
        file.close();
        ROS_INFO("gt_poses.size(): %ld", gt_poses.size());
        return gt_poses;
    }
    else
    {
        ROS_ERROR("ground pose file not found %s", path.c_str());
        exit(-1);
    }
}

cv::Mat DatasetLoader::readIntrinsicMat(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("calib file not found");
        exit(-1);
    }

    cv::Mat intrinsic_mat;

    std::string line;
    
    // 파일에서 각 줄을 읽어와서 처리
    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string label;
        ss >> label;
        
        if(label != "P2:")
            continue;

        double values[3][4];
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                if (!(ss >> values[i][j]))
                {
                    ROS_ERROR("reading calib file is failed");
                    exit(-1);
                }
            }
        }
    
        cv::Mat mat(3, 4, CV_64F, values);
        intrinsic_mat = mat(cv::Rect(0,0,3,3));
        // intrinsic_mat = (cv::Mat_<double>(3, 3) << mat.at<double>(0,0), mat.at<double>(0,1), mat.at<double>(0,2),
        //                                            mat.at<double>(1,0), mat.at<double>(1,1), mat.at<double>(1,2),
        //                                            mat.at<double>(2,0), mat.at<double>(2,1), mat.at<double>(2,2));
    }
    
    if(intrinsic_mat.empty())
    {
        ROS_ERROR("intrinsic_mat initialization is failed");
        exit(-1);
    }

    return intrinsic_mat.clone();
}

std::vector<double> DatasetLoader::getTimeSequence(const std::string& path)
{
    std::vector<double> times;
    std::ifstream file(path);
    
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << path << std::endl;
        return times;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        try {
            double time = std::stod(line);
            times.push_back(time);
        } catch (const std::exception& e) {
            std::cerr << "Error converting line to double: " << line << std::endl;
        }
    }
    
    file.close();
    return times;
}

#endif