/**
 * @file LogReader.h
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

#ifndef LOGREADER_H_
#define LOGREADER_H_

#ifdef WIN32
#  include <cstdint>
#endif
#include <string>
#if (defined WIN32) && (defined FAR)
#  undef FAR
#endif
#include <zlib.h>
#ifndef WIN32
#  include <poll.h>
#endif

// 这个文件并不是必要的
// #include <Utils/Img.h>
#include <Utils/Resolution.h>

// JPEG 解码器支持
#include "JPEGLoader.h"

/** @brief 记录文件读取器 */
class LogReader
{
    public:
        /**
         * @brief 构造函数
         * @param[in] file 记录文件路径
         * @param[in] flipColors 是否翻转图像RGB/BGR
         */
        LogReader(std::string file, bool flipColors)
         : flipColors(flipColors),
           timestamp(0),
           depth(0),
           rgb(0),
           currentFrame(0),
           decompressionBufferDepth(0),
           decompressionBufferImage(0),
           file(file),
           width(Resolution::getInstance().width()),
           height(Resolution::getInstance().height()),
           numPixels(width * height)
        {}

        /** @brief 空析构函数, 由子类实现 */
        virtual ~LogReader()
        {}

        /** @brief 正序获取下一帧的图像数据, 保存在缓冲区中 */
        virtual void getNext() = 0;

        /** 
         * @brief 获取当前记录文件中的帧数
         * @return int 帧数
         */
        virtual int getNumFrames() = 0;

        /**
         * @brief 记录文件中的帧数据是否全部读取完成
         * @return 是否
         */
        virtual bool hasMore() = 0;

        /**
         * @brief 如果逆序读取, 现在是否已经需要"倒带"转变成为正序读取?
         * @return true 
         * @return false 
         */
        virtual bool rewound() = 0;

        /** @brief 重置整个记录文件读取器 */
        virtual void rewind() = 0;

        /** @brief 倒序读取记录文件 */
        virtual void getBack() = 0;

        /**
         * @brief 快进到第 frame 帧
         * @param[in] frame 帧id
         */
        virtual void fastForward(int frame) = 0;

        /**
         * @brief 获取记录文件的路径
         * @return const std::string 路径
         */
        virtual const std::string getFile() = 0;

        /**
         * @brief 设置自动曝光/白平衡参数
         * @param[in] value 是否设置
         */
        virtual void setAuto(bool value) = 0;

        bool flipColors;                    ///< 是否翻转图像 RGB、BGR
        int64_t timestamp;                  ///< 帧的时间戳, 单位为 ns

        unsigned short * depth;             ///< 可以直接使用的深度图像首地址, 注意和下面的 decompressionBufferDepth 指针数据类型不同
        unsigned char * rgb;                ///< 可以直接使用的彩色图像首地址
        int currentFrame;                   ///< 当前帧的计数

    protected:
        Bytef * decompressionBufferDepth;   ///< 解压缩后可以直接使用的深度图像
        Bytef * decompressionBufferImage;   ///< 解压缩后可以直接使用的深度图像
        unsigned char * depthReadBuffer;    ///< 从外部文件中读取到的深度图像数据将会被暂时存储到这里
        unsigned char * imageReadBuffer;    ///< 从外部文件中读取到的彩色图像数据将会被暂时存储到这里
        int32_t depthSize;                  ///< 深度图像的数据长度(byte)
        int32_t imageSize;                  ///< 彩色图像的数据长度(byte)

        const std::string file;             ///< 记录文件的路径
        FILE * fp;                          ///< C 风格的文件指针, 用于指向打开的记录文件
        int32_t numFrames;                  ///< 记录文件中帧的个数
        int width;                          ///< 输入图像的宽度
        int height;                         ///< 输入图像的高度
        int numPixels;                      ///< 输入图像的像素数目

        JPEGLoader jpeg;                    ///< Jpeg 编解码器
};

#endif /* LOGREADER_H_ */
