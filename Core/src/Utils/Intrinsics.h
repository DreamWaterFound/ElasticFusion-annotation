/**
 * @file Intrinsics.h
 * @author guoqing (1337841346@qq.com)
 * @brief 相机内参对象, 包含 fx fy cx cy
 * @version 0.1
 * @date 2020-01-03
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

#ifndef INTRINSICS_H_
#define INTRINSICS_H_

// C
#include <cassert>
// windows DLL
#include "../Defines.h"

/** @brief 保存相机内参 fx fy cx cy 的类 */
class Intrinsics
{
    public:
        /**
         * @brief 使用给定的内参数, 获取内参类对象 静态成员函数
         * @param[in] fx 
         * @param[in] fy 
         * @param[in] cx 
         * @param[in] cy 
         * @return EFUSION_API const& getInstance 内参类对象的引用
         */
        EFUSION_API static const Intrinsics & getInstance(float fx = 0,float fy = 0,float cx = 0,float cy = 0);

        /**
         * @brief 获取相机内参 fx
         * @return const float& fx
         */
        const float & fx() const
        {
            return fx_;
        }

        /**
         * @brief 获取相机内参 fy
         * @return const float& fy
         */
        const float & fy() const
        {
            return fy_;
        }

        /**
         * @brief 获取相机内参 cx
         * @return const float& cx
         */
        const float & cx() const
        {
            return cx_;
        }

        /**
         * @brief 获取相机内参 cy
         * @return const float& cy
         */
        const float & cy() const
        {
            return cy_;
        }

    private:

        /**
         * @brief 给定相机内参的构造函数
         * @param[in] fx 
         * @param[in] fy 
         * @param[in] cx 
         * @param[in] cy 
         */
        Intrinsics(float fx, float fy, float cx, float cy)
         : fx_(fx),
           fy_(fy),
           cx_(cx),
           cy_(cy)
        {
            assert(fx != 0 && fy != 0 && "You haven't initialised the Intrinsics class!");
        }

        /// 相机内参
        const float fx_, fy_, cx_, cy_;
};

#endif /* INTRINSICS_H_ */
