/**
 * @file GroundTruthOdometry.h
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

#ifndef GROUNDTRUTHODOMETRY_H_
#define GROUNDTRUTHODOMETRY_H_

#ifdef WIN32
#  include <cstdint>
#endif 

// Eigen
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>

// STL
#include <iostream>
#include <fstream>
#include <map>

// 位姿计算和转换的小工具
#include <Utils/OdometryProvider.h>

/** @brief // ? */
class GroundTruthOdometry
{
    public:
        /**
         * @brief 构造函数
         * @param[in] filename 位姿真值文件
         */
        GroundTruthOdometry(const std::string & filename);

        /** @brief 析构函数 */
        virtual ~GroundTruthOdometry();

        Eigen::Matrix4f getTransformation(uint64_t timestamp);

        Eigen::MatrixXd getCovariance();

    private:

        /**
         * @brief 从外部文件中加载相机的运动真值
         * @param[in] filename 
         */
        void loadTrajectory(const std::string & filename);

        std::map<uint64_t,              // 时间戳
                 Eigen::Isometry3f,     // 位姿
                 std::less<int>, 
                 Eigen::aligned_allocator<
                    std::pair<const uint64_t, Eigen::Isometry3f> 
                    > 
                > camera_trajectory;        ///< 保存相机位姿的 vector


        uint64_t last_utime;                ///? 最近的什么时间戳?
};

#endif /* GROUNDTRUTHODOMETRY_H_ */
