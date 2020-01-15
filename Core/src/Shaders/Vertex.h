/**
 * @file Vertex.h
 * @author guoqing (1337841346@qq.com)
 * @brief 顶点类
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

#ifndef VERTEX_H_
#define VERTEX_H_

#include <Eigen/Core>

#include "../Defines.h"

/** @brief 顶点类 // ? 这个感觉完全没有内容, 为什么还要存在它? 只是为了提供 SIZE? */
class Vertex
{
    public:
        ///? 啥SIZE? 感觉像是每个顶点占用的字节大小
        EFUSION_API static const int SIZE;

    private:
        /** @brief 顶点的构造函数 */
        Vertex(){}
};


#endif /* VERTEX_H_ */
