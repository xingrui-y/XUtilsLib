#include <xutils/IOWrapper/tum_loader.h>

namespace xutils
{

TUMDataLoader::TUMDataLoader(std::string dir)
    : imageId(0), baseDir(dir)
{
    if (baseDir.back() != '/')
        baseDir += '/';
}

bool TUMDataLoader::loadAssociation(std::string file_name)
{
    std::ifstream file;
    file.open(baseDir + file_name, std::ios_base::in);

    double ts;
    std::string name_depth, name_image;

    while (file >> ts >> name_image >> ts >> name_depth)
    {
        imageFileList.push_back(name_image);
        depthFileList.push_back(name_depth);
        timeStampList.push_back(ts);
    }

    if (depthFileList.size() == 0)
    {
        std::cout << "Reading images failed, please check your directory.\n";
        return false;
    }
    else
    {
        printf("Total of %lu Images Loaded.\n", depthFileList.size());
        return true;
    }
}

int TUMDataLoader::findClosestIndex(const std::vector<double> &list, double time) const
{
    int idx = -1;
    double min_val = std::numeric_limits<double>::max();
    for (int i = 0; i < list.size(); ++i)
    {
        double d = std::abs(list[i] - time);
        if (d < min_val)
        {
            idx = i;
            min_val = d;
        }
    }

    return idx;
}

void TUMDataLoader::loadGroundTruth(std::string file_name)
{
    if (timeStampList.size() == 0)
    {
        printf("Please load images first!\n");
        return;
    }

    double ts;
    double tx, ty, tz, qx, qy, qz, qw;

    std::ifstream file;
    file.open(baseDir + file_name);

    for (int i = 0; i < 3; ++i)
    {
        std::string line;
        std::getline(file, line);
    }

    std::vector<double> ts_gt;
    std::vector<Sophus::SE3d> vgt;
    while (file >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw)
    {
        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalize();
        auto r = q.toRotationMatrix();
        auto t = Eigen::Vector3d(tx, ty, tz);
        Sophus::SE3d gt(r, t);

        ts_gt.push_back(ts);
        vgt.push_back(gt);
    }

    for (int i = 0; i < timeStampList.size(); ++i)
    {
        double time = timeStampList[i];
        int idx = findClosestIndex(ts_gt, time);
        timeStampListGT.push_back(ts_gt[idx]);
        poseListGT.push_back(vgt[idx]);
    }

    file.close();
    printf("Total of %lu Ground Truth Data Loaded.\n", poseListGT.size());
}

bool TUMDataLoader::loadNextImages(cv::Mat &depth, cv::Mat &image)
{
    if (imageId >= imageFileList.size())
        return false;

    std::string fullpath_image = baseDir + imageFileList[imageId];
    std::string fullpath_depth = baseDir + depthFileList[imageId];

    image = cv::imread(fullpath_image, cv::IMREAD_UNCHANGED);
    depth = cv::imread(fullpath_depth, cv::IMREAD_UNCHANGED);

    imageId++;
    return true;
}

void TUMDataLoader::saveTrajectory(std::vector<Sophus::SE3d> trajectory, std::string file_name) const
{
    std::ofstream file;
    std::string file_path = baseDir + file_name;
    file.open(file_path, std::ios_base::out);

    for (int i = 0; i < trajectory.size(); ++i)
    {
        if (i >= timeStampList.size())
            break;

        double ts = timeStampList[i];
        Sophus::SE3d &curr = trajectory[i];
        Eigen::Vector3d t = curr.translation();
        Eigen::Quaterniond q(curr.rotationMatrix());

        file << std::fixed
             << std::setprecision(4)
             << ts << " "
             << t(0) << " "
             << t(1) << " "
             << t(2) << " "
             << q.x() << " "
             << q.y() << " "
             << q.z() << " "
             << q.w() << std::endl;
    }

    file.close();
}

std::vector<Sophus::SE3d> TUMDataLoader::getPoseListGT() const
{
    return poseListGT;
}

float TUMDataLoader::getDepthScale() const
{
    return 1.f / 5000.f;
}

double TUMDataLoader::getTimeStamp() const
{
    return timeStampList[imageId - 1];
}

std::size_t TUMDataLoader::getImageId() const
{
    return imageId - 1;
}

Sophus::SE3d TUMDataLoader::getStartingPose() const
{
    double time = timeStampList[0];
    int idx = findClosestIndex(timeStampListGT, time);
    return poseListGT[idx];
}

} // namespace xutils