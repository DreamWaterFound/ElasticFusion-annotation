/**
 * @file ComputePack.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 装配好的着色管线对象
 * @version 0.1
 * @date 2020-01-15
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


#include "ComputePack.h"

const std::string ComputePack::NORM             = "NORM";               // 原始
const std::string ComputePack::FILTER           = "FILTER";             // 双边滤波后
const std::string ComputePack::METRIC           = "METRIC";             // 度量化的深度图, 每个像素表示实际距离
const std::string ComputePack::METRIC_FILTERED  = "METRIC_FILTERED";    // 双边滤波后度量化的深度图

// 构造函数
ComputePack::ComputePack(std::shared_ptr<Shader> program,               // GLSL 程序对象
                         pangolin::GlTexture * target)                  // GPU纹理
 : program(program),                                                    // GLSL 程序对象句柄
   renderBuffer(Resolution::getInstance().width(), Resolution::getInstance().height()), // 生成渲染缓存
   target(target)                                                       // 纹理对象
{
    frameBuffer.AttachColour(*target);      // 彩色信息绑定输出到纹理对象中
    frameBuffer.AttachDepth(renderBuffer);  // 深度信息绑定输出到渲染缓存中
}

// 析构函数
ComputePack::~ComputePack(){}

void ComputePack::compute(pangolin::GlTexture * input, const std::vector<Uniform> * const uniforms)
{
    input->Bind();

    frameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, renderBuffer.width, renderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program->Bind();

    if(uniforms)
    {
        for(size_t i = 0; i < uniforms->size(); i++)
        {
            program->setUniform(uniforms->at(i));
        }
    }

    glDrawArrays(GL_POINTS, 0, 1);

    frameBuffer.Unbind();

    program->Unbind();

    glPopAttrib();

    glFinish();
}
