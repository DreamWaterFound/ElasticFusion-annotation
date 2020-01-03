/**
 * @file Resolution.h
 * @author guoqing (1337841346@qq.com)
 * @brief 分辨率对象
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

#ifndef RESOLUTION_H_
#define RESOLUTION_H_

#include <cassert>

#include "../Defines.h"

/** @brief 分辨率对象 */
class Resolution
{
    public:
        /**
         * @brief 获取给定分辨率的分辨率对象实例, 静态函数
         * @param[in] width 给定宽度
         * @param[in] height 给定高度
         * @return EFUSION_API const& getInstance 分辨率对象实例
         */
        EFUSION_API static const Resolution & getInstance(int width = 0,int height = 0);

        /**
         * @brief 获取宽度
         * @return const int& 
         */
        const int & width() const
        {
            return imgWidth;
        }

        /**
         * @brief 获取高度
         * @return const int& 
         */
        const int & height() const
        {
            return imgHeight;
        }

        const int & cols() const
        {
            return imgWidth;
        }

        const int & rows() const
        {
            return imgHeight;
        }

        const int & numPixels() const
        {
            return imgNumPixels;
        }

    private:
        /**
         * @brief 构造函数
         * @param[in] width 给定宽度
         * @param[in] height 给定高度
         */
        Resolution(int width, int height)
         : imgWidth(width),
           imgHeight(height),
           imgNumPixels(width * height)
        {
            // 不允许为0x0的图像存在
            assert(width > 0 && height > 0 && "You haven't initialised the Resolution class!");
        }

        // 分辨率信息
        const int imgWidth;                 ///< 图像宽度
        const int imgHeight;                ///< 图像高度
        const int imgNumPixels;             ///< 图像中所包含的像素个数
};

#endif /* RESOLUTION_H_ */
