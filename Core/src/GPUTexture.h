/**
 * @file GPUTexture.h
 * @author guoqing (1337841346@qq.com)
 * @brief GPU 纹理对象
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

#ifndef GPUTEXTURE_H_
#define GPUTEXTURE_H_

// Pangolin 
#include <pangolin/pangolin.h>
// CUDA 
#include <driver_types.h>
#include <cuda_gl_interop.h>
#include <cuda_runtime_api.h>
// DLL
#include "Defines.h"

/** @brief GPU 纹理的类 */
class GPUTexture
{
    public:
        /**
         * @brief 构造函数
         * @param[in] width             纹理图像的宽度
         * @param[in] height            纹理图像的高度
         * @param[in] internalFormat    纹理的数据格式
         * @param[in] format            对纹理格式的处理
         * @param[in] dataType          每个通道的数据类型
         * @param[in] draw              // ? 是否绘制? 
         * @param[in] cuda              使用CUDA支持
         * @return EFUSION_API 
         */
        EFUSION_API GPUTexture(const int width,
                   const int height,
                   const GLenum internalFormat,
                   const GLenum format,
                   const GLenum dataType,
                   const bool draw = false,
                   const bool cuda = false);

        /** @brief 析构函数 */
        virtual ~GPUTexture();

        ///? 几个字符串, 标记当前纹理中存储的是什么内容, 用于在 Pangolin 中指示显示到的 Viewer 的 id
        EFUSION_API static const std::string RGB, DEPTH_RAW, DEPTH_FILTERED, DEPTH_METRIC, DEPTH_METRIC_FILTERED, DEPTH_NORM;

        
        pangolin::GlTexture * texture;          ///< OpenGL 中的纹理对象句柄
        cudaGraphicsResource * cudaRes;         ///? OpenGL 纹理对象注册到CUDA中后的资源句柄

        const bool draw;                        ///? 是否绘制? 还是是否可以绘制? 感觉这个纹理也会存储一些不可绘制的对象, 比如顶点, 比如法向

    private:
        /** @brief 无参构造函数 */
        GPUTexture() : texture(0), cudaRes(0), draw(false), width(0), height(0), internalFormat(0), format(0), dataType(0) {}
        const int width;                        ///< 图像宽度
        const int height;                       ///< 图像高度
        const GLenum internalFormat;            ///< 纹理本身的数据格式
        const GLenum format;                    ///< 对纹理格式的处理
        const GLenum dataType;                  ///< 每个通道的数据类型
};

#endif /* GPUTEXTURE_H_ */
