/**
 * @file Img.h
 * @author guoqing (1337841346@qq.com)
 * @brief 图像对象
 * @version 0.1
 * @date 2020-01-10
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

#ifndef UTILS_IMG_H_
#define UTILS_IMG_H_

// Eigen
#include <Eigen/Core>

/**
 * @brief 图像对象
 * @tparam T ElasticFusion 中将图像本身看做成为一个 Eigen 格式的矩阵, 这里的模板参数 T 就是这个矩阵的数据类型
 */
template <class T>
class Img
{
    public:
        /**
         * @brief 构造函数, 构造空图像
         * @param[in] rows 图像的行数
         * @param[in] cols 图像的列数
         */
        Img(const int rows, const int cols)
         : rows(rows),                                          // 图像行数
           cols(cols),                                          // 图像列数
           data(new unsigned char[rows * cols * sizeof(T)]),    // 立即分配空间
           owned(true)                                          // 这个图像数据空间是自己生成的, 所以也要由自己负责删除
        {}

        /**
         * @brief 构造函数, 给定图像数据构造图像
         * @note 这里不会分配新的图像数据空间, 而是直接共享传入的图像数据区
         * @param[in] rows 图像的行数
         * @param[in] cols 图像的列数
         * @param[in] data 图像数据, 头指针
         */
        Img(const int rows, const int cols, T * data)
         : rows(rows),                                          // 图像行数
           cols(cols),                                          // 图像列数
           data((unsigned char *)data),                         // 共享传入的数据区
           owned(false)                                         // 这个图像数据空间不是自己生成的, 所以自己析构的时候不要删除
        {}

        virtual ~Img()
        {
            // 如果这个图像数据空间是自己生成的, 所以也要由自己负责删除
            if(owned)
            {
                delete [] data;
            }
        }

        const int rows;                             ///< 图像的行数
        const int cols;                             ///< 图像的列数
        unsigned char * data;                       ///< 图像的数据区首指针
        const bool owned;                           ///< 图像的数据区是否是当前类对象构造的时候分配的, 用于在析构时是否要释放图像数据区

        /**
         * @brief 获取图像中某个像素处的数据, 按照一维索引; 数据可读可写
         * @tparam V        以何种数据格式读取
         * @param[in] i     像素在整个数据区的一维索引
         * @return V&       引用
         */
        template<typename V> inline
        V & at(const int i)
        {
            return ((V*)data)[i];
        }

        /**
         * @brief 获取图像中某个像素处的数据, 对原数据可读可写
         * @tparam V            读取的数据格式
         * @param[in] row       像素所在行数
         * @param[in] col       像素所在列数
         * @return V&           引用
         */
        template<typename V> inline
        V & at(const int row, const int col)
        {
            return ((V*)data)[cols * row + col];
        }

        /**
         * @brief 获取图像中某个像素处的数据; 不同的这个是只读的
         * @tparam V            读取的数据格式
         * @param[in] row       像素所在行数
         * @param[in] col       像素所在列数
         * @return V&           引用
         */
        template<typename V> inline const
        V & at(const int row, const int col) const
        {
            return ((const V*)data)[cols * row + col];
        }
};

#endif /* UTILS_IMG_H_ */
