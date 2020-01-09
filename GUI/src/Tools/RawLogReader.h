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

// 分辨率对象
#include <Utils/Resolution.h>
// 时间统计对象
#include <Utils/Stopwatch.h>

// Pangolin
#include <pangolin/utils/file_utils.h>

// 父类
#include "LogReader.h"

// C & C++ STL
#include <cassert>
#include <zlib.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stack>


/* 记录文件的数据格式: 
 * 0x..00   int32_t     当前数据集中的帧数
 * 0x..04   int64_t     第0帧数据的时间戳
 * 0x..0C   int32_t         -深度图像数据长度 depthSize
 * 0x..10   int32_t         -彩色图像数据长度 imageSize
 * 0x..14   depthSize       -深度图像数据
 * ..       imageSize       -彩色图像数据(不一定有)
 * ..       int64_t     第1帧数据的时间戳
*/


/** @brief 原始记录文件读取器, 这个读取器负责读取原始记录文件 */
class RawLogReader : public LogReader
{
    public:
    
        /**
         * @brief 构造函数
         * @param[in] file 记录文件位置
         * @param[in] flipColors 是否要翻转图像
         */
        RawLogReader(std::string file, bool flipColors);

        /** @brief  析构函数, 释放缓冲区内容, 关闭文件 */
        virtual ~RawLogReader();

        /** @brief 获取下一帧图像数据, 保存到缓冲区中 */
        void getNext();

        /** @brief 设置文件指针返回到上一帧图像开始的位置 */
        void getBack();

        /**
         * @brief 获取获取记录文件中的帧的个数
         * @return int 帧的个数
         */
        int getNumFrames();

        /* @brief 判断记录文件中的内容是否已经读取完毕 */
        bool hasMore();

        /**
         * @brief 判断是否需要倒带 -- 即记录文件倒序播放到头后, 再正序播放
         * @return 是否
         */
        bool rewound();

        /** @brief 重新设置记录文件的读取 */
        void rewind();

        /**
         * @brief 快进到第 frame 帧
         * @param[in] frame 要快进到的帧id
         */
        void fastForward(int frame);

        /**
         * @brief 获取记录文件路径
         * @return const std::string 记录文件路径
         */
        const std::string getFile();

        /**
         * @brief 自动曝光/白平衡参数设置, 对于记录文件无法设置, 空函数
         * @param[in] value 是否
         */
        void setAuto(bool value);

        std::stack<int> filePointers;               ///< 存储每一帧图像的数据相对于文件头部的偏移量

    private:
        /** @brief 获取真正的帧数据到缓冲区中 */
        void getCore();
};

#endif /* RAWLOGREADER_H_ */
