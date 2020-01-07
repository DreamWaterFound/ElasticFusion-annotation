/**
 * @file RawLogReader.h
 * @author guoqing (1337841346@qq.com)
 * @brief 记录文件读取器
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

#ifndef RAWLOGREADER_H_
#define RAWLOGREADER_H_

#include <Utils/Resolution.h>
#include <Utils/Stopwatch.h>
#include <pangolin/utils/file_utils.h>

#include "LogReader.h"

#include <cassert>
#include <zlib.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stack>


/** @brief 原始记录文件读取器 */
// ? 这个读取器和父类有什么不同?
class RawLogReader : public LogReader
{
    public:
    
        /**
         * @brief 构造函数
         * @param[in] file 记录文件位置
         * @param[in] flipColors 是否要翻转图像
         */
        RawLogReader(std::string file, bool flipColors);

        virtual ~RawLogReader();

        void getNext();

        void getBack();

        int getNumFrames();

        /* @brief 判断记录文件中的内容是否已经读取完毕 */
        bool hasMore();

        bool rewound();

        /** @brief 重新读取记录文件 */
        void rewind();

        void fastForward(int frame);

        /**
         * @brief 获取记录文件路径
         * @return const std::string 记录文件路径
         */
        const std::string getFile();

        void setAuto(bool value);

        std::stack<int> filePointers;               ///? 不知道存储啥的文件指针的堆栈

    private:
        void getCore();
};

#endif /* RAWLOGREADER_H_ */
