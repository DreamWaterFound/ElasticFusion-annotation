/**
 * @file GPUTexture.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief GPU中的纹理对象
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
 
#include "GPUTexture.h"

const std::string GPUTexture::RGB                       = "RGB";                        // 原始彩色图像
const std::string GPUTexture::DEPTH_RAW                 = "DEPTH";                      // 原始深度图像
const std::string GPUTexture::DEPTH_FILTERED            = "DEPTH_FILTERED";             // 双边滤波后的深度图像
const std::string GPUTexture::DEPTH_METRIC              = "DEPTH_METRIC";               // 度量表示的深度图像(float, 像素值表示距离)
const std::string GPUTexture::DEPTH_METRIC_FILTERED     = "DEPTH_METRIC_FILTERED";      // 度量表示的双边滤波后的深度图像(float, 像素值表示距离)
const std::string GPUTexture::DEPTH_NORM                = "DEPTH_NORM";                 // ? 根据深度图像计算得到的法向图

// 构造函数
GPUTexture::GPUTexture(const int width,
                       const int height,
                       const GLenum internalFormat,
                       const GLenum format,
                       const GLenum dataType,
                       const bool draw,
                       const bool cuda)
 : texture(new pangolin::GlTexture(width, height, internalFormat, draw, 0, format, dataType)),
   draw(draw),
   width(width),
   height(height),
   internalFormat(internalFormat),
   format(format),
   dataType(dataType)
{
    if(cuda)
    {
        // 如果使用 OpenGL 和 CUDA 互操作, 这里就需要在CUDA中注册 OpenGL 的纹理对象
        cudaGraphicsGLRegisterImage(&cudaRes, texture->tid, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsReadOnly);
    }
    else
    {
        // 不使用的话设置空指针, 这里直接设置成为0了
        cudaRes = 0;
    }
}

// 析构函数
GPUTexture::~GPUTexture()
{
    // 释放OpenGL 纹理对象
    if(texture)
    {
        delete texture;
    }

    // 取消 CUDA 中的注册
    if(cudaRes)
    {
        cudaGraphicsUnregisterResource(cudaRes);
    }
}
