/**
 * @file ComputePack.h
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

#ifndef COMPUTEPACK_H_
#define COMPUTEPACK_H_

#include "Shaders.h"
#include "../Utils/Resolution.h"
#include "Uniform.h"
#include <pangolin/gl/gl.h>

/** @brief 装配好的着色管线对象
 *  @note 用于辅助 GLSL 程序进行计算的, 设置着色器计算的输入内容和输出内容(均以纹理的形式展现) */
class ComputePack
{
    public:
        /**
         * @brief 构造函数
         * @param[in] program GLSL 程序对象
         * @param[in] target  GPU纹理对象
         */
        ComputePack(std::shared_ptr<Shader> program,
                    pangolin::GlTexture * target);

        /** @brief 析构函数 */
        virtual ~ComputePack();

        static const std::string NORM, FILTER, METRIC, METRIC_FILTERED;

        void compute(pangolin::GlTexture * input, const std::vector<Uniform> * const uniforms = 0);

    private:
        std::shared_ptr<Shader> program;            ///< GLSL 程序对象
        pangolin::GlRenderBuffer renderBuffer;      ///? 渲染缓存
        pangolin::GlTexture * target;               ///< 纹理对象
        pangolin::GlFramebuffer frameBuffer;        ///? 帧缓存
};

#endif /* COMPUTEPACK_H_ */
