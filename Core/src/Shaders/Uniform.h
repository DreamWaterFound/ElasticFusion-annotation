/**
 * @file Uniform.h
 * @author guoqing (1337841346@qq.com)
 * @brief 管理 uniform 数据块, 负责给 GLSL 程序传递数据的类, 统一了不同数据之间的接口
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
 
#ifndef UNIFORM_H_
#define UNIFORM_H_

// C++ STL
#include <string>
// Eigen
#include <Eigen/Core>

/** @brief 统一各种数据类型, 方便给 GLSL 程序传递参数的类 */
class Uniform
{
    public:
        /**
         * @brief 构造函数 - int
         * @param[in] id 变量的字符串名字, 也作为其唯一的标识符
         * @param[in] v  变量内容
         */
        Uniform(const std::string & id, const int & v)
         : id(id),
           i(v),
           t(INT)
        {}

        /**
         * @brief 构造函数 - float
         * @param[in] id 变量的字符串名字, 也作为其唯一的标识符
         * @param[in] v  变量内容
         */
        Uniform(const std::string & id, const float & v)
         : id(id),
           f(v),
           t(FLOAT)
        {}

        /**
         * @brief 构造函数 - Vector2f
         * @param[in] id 变量的字符串名字, 也作为其唯一的标识符
         * @param[in] v  变量内容
         */
        Uniform(const std::string & id, const Eigen::Vector2f & v)
         : id(id),
           v2(v),
           t(VEC2)
        {}

        /**
         * @brief 构造函数 - Vector3f
         * @param[in] id 变量的字符串名字, 也作为其唯一的标识符
         * @param[in] v  变量内容
         */
        Uniform(const std::string & id, const Eigen::Vector3f & v)
         : id(id),
           v3(v),
           t(VEC3)
        {}

        /**
         * @brief 构造函数 - Vector4f
         * @param[in] id 变量的字符串名字, 也作为其唯一的标识符
         * @param[in] v  变量内容
         */
        Uniform(const std::string & id, const Eigen::Vector4f & v)
         : id(id),
           v4(v),
           t(VEC4)
        {}

        /**
         * @brief 构造函数 - Matrix4f
         * @param[in] id 变量的字符串名字, 也作为其唯一的标识符
         * @param[in] v  变量内容
         */
        Uniform(const std::string & id, const Eigen::Matrix4f & v)
         : id(id),
           m4(v),
           t(MAT4)
        {}

        std::string id;         ///< 变量名的字符串描述, 也是作为其标志

        int i;                  ///< 整型
        float f;                ///< 浮点型
        Eigen::Vector2f v2;     ///< 2x1 向量
        Eigen::Vector3f v3;     ///< 3x1 向量
        Eigen::Vector4f v4;     ///< 4x1 向量
        Eigen::Matrix4f m4;     ///< 4x4 矩阵

        // ==== 当前类支持的数据类型 ====
        enum Type
        {
            INT,
            FLOAT,
            VEC2,
            VEC3,
            VEC4,
            MAT4,
            NONE
        };

        Type t;                 ///< 数据类型
};


#endif /* UNIFORM_H_ */
