#ifndef XUTILS_TUM_DATA_LOADER
#define XUTILS_TUM_DATA_LOADER

#include <iostream>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

namespace xutils
{

class TUMDataLoader
{
public:
    TUMDataLoader(std::string baseDir);
    bool loadAssociation(std::string file_name);
    void loadGroundTruth(std::string file_name);
    bool loadNextImages(cv::Mat &depth, cv::Mat &image);

    void saveTrajectory(std::vector<Sophus::SE3d> trajectory, std::string file_name) const;
    double getTimeStamp() const;
    Sophus::SE3d getStartingPose() const;
    std::size_t getImageId() const;
    std::vector<Sophus::SE3d> getPoseListGT() const;
    float getDepthScale() const;

private:
    int findClosestIndex(const std::vector<double> &list, double time) const;
    std::size_t imageId;
    std::string baseDir;
    std::vector<double> timeStampList;
    std::vector<std::string> imageFileList;
    std::vector<std::string> depthFileList;
    std::vector<double> timeStampListGT;
    std::vector<Sophus::SE3d> poseListGT;
};

} // namespace xutils

#endif