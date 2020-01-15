/**
 * @file Ferns.h
 * @author guoqing (1337841346@qq.com)
 * @brief 随机蕨数据库
 * @version 0.1
 * @date 2020-01-09
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

#ifndef FERNS_H_
#define FERNS_H_

// C++
#include <random>
#include <vector>
#include <limits>

// Eigen
#include <Eigen/Core>
#include <Eigen/LU>

// 分辨率对象
#include "Utils/Resolution.h"
// 相机内参对象
#include "Utils/Intrinsics.h"
// 视觉里程计对象
#include "Utils/RGBDOdometry.h"
// 利用 GLSL 实现的图像大小缩放对象
#include "Shaders/Resize.h"

/** @brief 随机蕨数据库 */
class Ferns
{
    public:
        /**
         * @brief 构造函数
         * @param[in] n                 蕨的棵数
         * @param[in] maxDepth          深度切断值
         * @param[in] photoThresh       Global loop closure photometric threshold
         */
        Ferns(int n, int maxDepth, const float photoThresh);


        virtual ~Ferns();

        bool addFrame(GPUTexture * imageTexture, GPUTexture * vertexTexture, GPUTexture * normalTexture, const Eigen::Matrix4f & pose, int srcTime, const float threshold);

        /** @brief 由于 Local/Global Loop 带来的约束信息 */
        class SurfaceConstraint
        {
            public:
                SurfaceConstraint(const Eigen::Vector4f & sourcePoint,
                                  const Eigen::Vector4f & targetPoint)
                 : sourcePoint(sourcePoint),
                   targetPoint(targetPoint)
                {}

                Eigen::Vector4f sourcePoint;            ///< 源
                Eigen::Vector4f targetPoint;            ///< 目标 //? 为什么是 Vecto4f? 第四维恒为1吗?
        };

        Eigen::Matrix4f findFrame(std::vector<SurfaceConstraint> & constraints,
                                  const Eigen::Matrix4f & currPose,
                                  GPUTexture * vertexTexture,
                                  GPUTexture * normalTexture,
                                  GPUTexture * imageTexture,
                                  const int time,
                                  const bool lost);

        /** @brief 表示每一个"蕨", 嵌套类  // ? 对应着一对采样点? */
        class Fern
        {
            public:
                /** @brief 空构造函数 */
                Fern(){}

                Eigen::Vector2i pos;            ///< 点的位置
                Eigen::Vector4i rgbd;           ///?
                std::vector<int> ids[16];       ///?
        };

        std::vector<Fern> conservatory;         ///< 保存所有蕨的"温室", 其实就是所有蕨的组合(森林)

        /** @brief 帧类别, 用于随机蕨数据库 */
        class Frame
        {
            public:
                Frame(int n,
                      int id,
                      const Eigen::Matrix4f & pose,
                      const int srcTime,
                      const int numPixels,
                      unsigned char * rgb = 0,
                      Eigen::Vector4f * verts = 0,
                      Eigen::Vector4f * norms = 0)
                 : goodCodes(0),
                   id(id),
                   pose(pose),
                   srcTime(srcTime),
                   initRgb(rgb),
                   initVerts(verts),
                   initNorms(norms)
                {
                    codes = new unsigned char[n];

                    if(rgb)
                    {
                        this->initRgb = new unsigned char[numPixels * 3];
                        memcpy(this->initRgb, rgb, numPixels * 3);
                    }

                    if(verts)
                    {
                        this->initVerts = new Eigen::Vector4f[numPixels];
                        memcpy(this->initVerts, verts, numPixels * sizeof(Eigen::Vector4f));
                    }

                    if(norms)
                    {
                        this->initNorms = new Eigen::Vector4f[numPixels];
                        memcpy(this->initNorms, norms, numPixels * sizeof(Eigen::Vector4f));
                    }
                }

                virtual ~Frame()
                {
                    delete [] codes;

                    if(initRgb)
                        delete [] initRgb;

                    if(initVerts)
                        delete [] initVerts;

                    if(initNorms)
                        delete [] initNorms;
                }

                unsigned char * codes;
                int goodCodes;
                const int id;
                Eigen::Matrix4f pose;
                const int srcTime;
                unsigned char * initRgb;
                Eigen::Vector4f * initVerts;
                Eigen::Vector4f * initNorms;
        };

        std::vector<Frame*> frames;                         ///< 随机蕨数据库中的图像帧对象

        const int num;                                      ///< 蕨的棵数
        std::mt19937 random;                                ///< 高性能的随机数发生器, C++ STL 提供
        const int factor;                                   ///< 随机蕨采样的图像相对于原图像的缩放系数, 这里直接进行了 1/8 下采样 
        const int width;                                    ///< 采样图像的宽度
        const int height;                                   ///< 采样图像的高度
        const int maxDepth;                                 ///< 深度切断值
        const float photoThresh;                            ///< Global loop closure photometric threshold
        std::uniform_int_distribution<int32_t> widthDist;   ///< 用于随机蕨在宽度维度上进行随机采样的一维均匀分布对象
        std::uniform_int_distribution<int32_t> heightDist;  ///< 用于随机蕨在高度维度上进行随机采样的一维均匀分布对象
        std::uniform_int_distribution<int32_t> rgbDist;     ///< RGB强度值(0-255)的均匀分布对象 // ? 拿来做什么用?
        std::uniform_int_distribution<int32_t> dDist;       ///< 深度值最低从 400 开始, 到切断距离的均匀分布对象

        int lastClosest;                                    ///? 最近的和当前帧匹配的帧在 frames 中的id?
        const unsigned char badCode;                        ///< Bad code // ? 表示什么的 bad  code?
        RGBDOdometry rgbd;                                  ///? 做什么用的里程计对象? 感觉好像是在根据外观决定 Global Loop 后, 进一步决定几何机构是否匹配的时候使用的

    private:
        void generateFerns();

        float blockHD(const Frame * f1, const Frame * f2);
        float blockHDAware(const Frame * f1, const Frame * f2);

        float photometricCheck(const Img<Eigen::Vector4f> & vertSmall,
                               const Img<Eigen::Matrix<unsigned char, 3, 1>> & imgSmall,
                               const Eigen::Matrix4f & estPose,
                               const Eigen::Matrix4f & fernPose,
                               const unsigned char * fernRgb);

        // 顶点
        GPUTexture vertFern;            ///? 
        GPUTexture vertCurrent;         ///? 当前图像的顶点纹理

        // 法向
        GPUTexture normFern;            ///?
        GPUTexture normCurrent;         ///? 当前图像的法向纹理

        // 颜色
        GPUTexture colorFern;           ///?
        GPUTexture colorCurrent;        ///? 当前图像的彩色纹理

        Resize resize;                  ///< 使用GLSL实现的resize对象

        Img<Eigen::Matrix<unsigned char, 3, 1>> imageBuff;      ///< 彩色图像缓冲
        Img<Eigen::Vector4f> vertBuff;                          ///< 顶点缓冲
        Img<Eigen::Vector4f> normBuff;                          ///< 法向缓冲
};

#endif /* FERNS_H_ */
