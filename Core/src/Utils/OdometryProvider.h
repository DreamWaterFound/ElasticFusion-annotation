/**
 * @file OdometryProvider.h
 * @author guoqing (1337841346@qq.com)
 * @brief 一个小工具, 目测是用于进行旋转矩阵, 李群李代数的转换的
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

#ifndef ODOMETRYPROVIDER_H_
#define ODOMETRYPROVIDER_H_

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// 平台相关的关于浮点数的一些性质
#include <float.h>

/** @brief 里程计信息计算辅助工具, 静态类 */
class OdometryProvider
{
    public:
        /** @brief 空构造函数 */
        OdometryProvider()
        {}

        /** @brief 空析构函数 */
        virtual ~OdometryProvider()
        {}

        /**
         * @brief 旋转向量 => 旋转矩阵 转换的函数 静态成员函数
         * @param[in] src 旋转向量
         * @return Eigen::Matrix<double, 3, 3, Eigen::RowMajor> 
         */
        static inline Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rodrigues(const Eigen::Vector3d & src)
        {
            // step 1 数据准备
            // 存储转换结果
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> dst = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Identity();

            double rx, ry, rz,      // 旋转轴
                   theta;           // 旋转角度

            // step 2 从旋转向量中获取旋转轴和转角大小
            rx = src(0);
            ry = src(1);
            rz = src(2);
            // 转角大小就是旋转向量的模长
            theta = src.norm();

            // step 3 如果不为0旋转
            if(theta >= DBL_EPSILON)
            {
                // 单位阵, 第一项矩阵因子
                const double I[] = { 1, 0, 0, 
                                     0, 1, 0, 
                                     0, 0, 1 };

                // 参考十四讲:
                double c = cos(theta);          // 第一项标量因子
                double s = sin(theta);          // 第三项标量因子
                double c1 = 1. - c;             // 第二项标量因子

                // 对旋转向量归一化 - 能够经过if的判断, 那么在这里就是 1./theta 了吧
                double itheta = theta ? 1./theta : 0.;
                rx *= itheta; ry *= itheta; rz *= itheta;

                // (旋转向量) x (旋转向量)^T, 第二项矩阵因子
                double rrt[] = { rx*rx, rx*ry, rx*rz,
                                 rx*ry, ry*ry, ry*rz, 
                                 rx*rz, ry*rz, rz*rz };
                // 旋转向量的反对称矩阵, 第三项矩阵因子
                double _r_x_[] = {  0, -rz,  ry, 
                                   rz,   0, -rx,
                                  -ry,  rx,   0 };

                // 保存计算结果, 之所以不保存到 dst 中是因为我们希望按行存储
                double R[9];
                // 现在 rodrigues 公式中的所有常数因子和矩阵都计算完成了, 那么接下来就对矩阵因子中的每一项进行乘法和求和计算就可以得到最终的结果
                for(int k = 0; k < 9; k++)
                {
                    R[k] = c*I[k] + c1*rrt[k] + s*_r_x_[k];
                }
                // 按行存, 组织计算结果
                memcpy(dst.data(), &R[0], sizeof(Eigen::Matrix<double, 3, 3, Eigen::RowMajor>));
            }
            return dst;
        }

        /**
         * @brief 计算 SE3 更新后的数值
         * @param[in&out]  resultRt  输入&输出, 对位姿的更新量, 以及计算完成成的Eigen矩阵形式组织的计算结果
         * @param[in]      result    se3
         * @param[out]     rgbOdom   Eigen::Isometry3f 格式的计算结果
         */
        static inline void computeUpdateSE3(Eigen::Matrix<double, 4, 4, Eigen::RowMajor> & resultRt, const Eigen::Matrix<double, 6, 1> & result, Eigen::Isometry3f & rgbOdom)
        {
            // for infinitesimal(无穷小) transformation
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Rt = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>::Identity();

            // 从 se3 获取平移向量
            Eigen::Vector3d rvec(result(3), result(4), result(5));

            // 从 se3 获取旋转向量对应的旋转矩阵
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R = rodrigues(rvec);

            // 构造普通 Eigen 形式的矩阵
            Rt.topLeftCorner(3, 3) = R;
            Rt(0, 3) = result(0);
            Rt(1, 3) = result(1);
            Rt(2, 3) = result(2);

            // Update
            resultRt = Rt * resultRt;

            // 组装 Eigen::Isometry3f 格式的计算结果
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotation = resultRt.topLeftCorner(3, 3);
            rgbOdom.setIdentity();
            // eval() 函数用于解决混淆问题, 参考[https://www.cnblogs.com/houkai/p/6349990.html]
            rgbOdom.rotate(rotation.cast<float>().eval());
            // topRightCorner() 取屏幕左上角的3行1列
            rgbOdom.translation() = resultRt.cast<float>().eval().topRightCorner(3, 1);
        }
};

#endif /* ODOMETRYPROVIDER_H_ */
