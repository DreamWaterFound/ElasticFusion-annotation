/**
 * @file GroundTruthOdometry.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 轨迹真值对象
 * @version 0.1
 * @date 2020-01-06
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include "GroundTruthOdometry.h"

// 构造函数, 从给定的文件中加载相机路径
GroundTruthOdometry::GroundTruthOdometry(const std::string & filename)
 : last_utime(0)
{
    loadTrajectory(filename);
}

// 空析构函数
GroundTruthOdometry::~GroundTruthOdometry()
{

}

// 从给定的文件中加载相机轨迹文件路径
void GroundTruthOdometry::loadTrajectory(const std::string & filename)
{
    // step 0 打开文件
    std::ifstream file;
    std::string line;
    file.open(filename.c_str());

    while (!file.eof())
    {
        // step 1 读取其中一行的数据
        unsigned long long int utime;       // 时间戳
        float x, y, z,                      // 位移
              qx, qy, qz, qw;               // 旋转
        std::getline(file, line);
        int n = sscanf(line.c_str(), "%llu,%f,%f,%f,%f,%f,%f,%f", &utime, &x, &y, &z, &qx, &qy, &qz, &qw);

        // step 2 验证: 文件没有结束, 并且的确得到了8个数据
        if(file.eof())
            break;

        assert(n == 8);

        // step 3 构造当前帧的位姿(Eigen格式)并加入到
        Eigen::Quaternionf q(qw, qx, qy, qz);
        Eigen::Vector3f t(x, y, z);

        Eigen::Isometry3f T;
        T.setIdentity();
        T.pretranslate(t).rotate(q);
        camera_trajectory[utime] = T;
    }
}

// 获取指定时间戳时, 相机位姿 Tcw
Eigen::Matrix4f GroundTruthOdometry::getTransformation(uint64_t timestamp)
{
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    // 不是第一次读取
    if(last_utime != 0)
    {
        // 找
        std::map<uint64_t, Eigen::Isometry3f>::const_iterator it = camera_trajectory.find(last_utime);
        if (it == camera_trajectory.end())
        {
            // 没找到? 就返回单位阵
            last_utime = timestamp;
            return pose;
        }
        
        //Poses are stored in the file in iSAM basis, undo it -- 有一个格式上的转化
        Eigen::Matrix4f M;
        M <<  0,  0, 1, 0,
             -1,  0, 0, 0,
              0, -1, 0, 0,
              0,  0, 0, 1;

        // ? 但是这里给的位姿是在原始相机位姿文件中给的坐标系表示的, 而下面返回的位姿却是在 EasticFusion 起始帧的相机坐标系下表示的?
        pose = M.inverse() * camera_trajectory[timestamp] * M;
    }
    else
    {
        // 是第一次读取相机的位姿
        std::map<uint64_t, Eigen::Isometry3f>::const_iterator it = camera_trajectory.find(timestamp);
        // 注意, 这里获得的一般不是单位阵, 但是后面返回pose却是单位阵
        Eigen::Isometry3f ident = it->second;
        pose = Eigen::Matrix4f::Identity();
        // 记录这个 0 对应的相机位姿, 相当于以后所有的返回的相机位姿真值是在 ElasticFusion 自己坐标系下的表示
        camera_trajectory[last_utime] = ident;
    }

    // Update
    last_utime = timestamp;

    return pose;
}

// 获取相机位姿的协方差, 恒对角阵
Eigen::MatrixXd GroundTruthOdometry::getCovariance()
{
    Eigen::MatrixXd cov(6, 6);
    cov.setIdentity();
    // 平移
    cov(0, 0) = 0.1;
    cov(1, 1) = 0.1;
    cov(2, 2) = 0.1;
    // 旋转
    cov(3, 3) = 0.5;
    cov(4, 4) = 0.5;
    cov(5, 5) = 0.5;
    return cov;
}
