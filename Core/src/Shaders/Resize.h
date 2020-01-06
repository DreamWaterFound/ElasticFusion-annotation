/**
 * @file Resize.h
 * @author guoqing (1337841346@qq.com)
 * @brief 对图像进行快捷缩放的类
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
 
#ifndef RESIZE_H_
#define RESIZE_H_

// GLSL 支持
#include "Shaders.h"
// ?
#include "Uniform.h"
// 分辨率对象
#include "../Utils/Resolution.h"
// 相机内参对象
#include "../Utils/Intrinsics.h"
// ? GPU 纹理? 
#include "../GPUTexture.h"
// ?
#include "../Utils/Img.h"

// DLL 相关定义
#include "../Defines.h"

/** @brief 缩放图像的对象 */
class Resize
{
    public:
        /**
         * @brief 构造函数
         * @param[in] srcWidth      源图像宽度
         * @param[in] srcHeight     源图像高度
         * @param[in] destWidth     目标图像宽度
         * @param[in] destHeight    目标图像高度
         */
        EFUSION_API Resize(int srcWidth,
               int srcHeight,
               int destWidth,
               int destHeight);

        /** @brief 析构函数 */
        virtual ~Resize();

        void image(GPUTexture * source, Img<Eigen::Matrix<unsigned char, 3, 1>> & dest);
        void vertex(GPUTexture * source, Img<Eigen::Vector4f> & dest);
        void time(GPUTexture * source, Img<unsigned short> & dest);

        GPUTexture imageTexture;            ///< 彩色图像纹理
        GPUTexture vertexTexture;           ///< 顶点伪装纹理
        GPUTexture timeTexture;             ///? 时间戳伪装纹理

        std::shared_ptr<Shader>     imageProgram;           ///< 缩放图像的 Shader, 直接将缩放图像的工作伪装成为了着色器的工作
        pangolin::GlRenderBuffer    imageRenderBuffer;      ///< 渲染 buffer
        pangolin::GlFramebuffer     imageFrameBuffer;       ///< 帧   buffer

        std::shared_ptr<Shader>     vertexProgram;          ///? 缩放/处理顶点的 Shader
        pangolin::GlRenderBuffer    vertexRenderBuffer;     ///< 渲染 buffer
        pangolin::GlFramebuffer     vertexFrameBuffer;      ///< 帧   buffer

        std::shared_ptr<Shader>     timeProgram;            ///? 缩放/处理时间戳的 Shader
        pangolin::GlRenderBuffer    timeRenderBuffer;       ///< 渲染 buffer
        pangolin::GlFramebuffer     timeFrameBuffer;        ///< 帧   buffer
};

#endif /* RESIZE_H_ */
