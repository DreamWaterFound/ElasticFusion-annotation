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
#include <Utils/Img.h>
#include <Utils/Resolution.h>

#include "JPEGLoader.h"

/** @brief 记录文件读取器 */
class LogReader
{
    public:
        /**
         * @brief 构造函数
         * @param[in] file 记录文件路径
         * @param[in] flipColors 是否翻转图像
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

        virtual ~LogReader()
        {}

        virtual void getNext() = 0;

        virtual int getNumFrames() = 0;

        virtual bool hasMore() = 0;

        virtual bool rewound() = 0;

        virtual void rewind() = 0;

        virtual void getBack() = 0;

        virtual void fastForward(int frame) = 0;

        virtual const std::string getFile() = 0;

        virtual void setAuto(bool value) = 0;

        bool flipColors;                ///< 是否左右翻转图像
        int64_t timestamp;              ///? 起始时间戳?

        unsigned short * depth;         ///?
        unsigned char * rgb;            ///?
        int currentFrame;               ///?

    protected:
        Bytef * decompressionBufferDepth;   ///? 目测是解压得到深度图像缓存的地方
        Bytef * decompressionBufferImage;   ///? 目测是解压得到彩色图像缓存的地方
        unsigned char * depthReadBuffer;    ///< 读取深度图像的时候, 一张图像的缓冲区头指针
        unsigned char * imageReadBuffer;    ///< 读取彩色图像的时候, 一张图像的缓冲区头指针
        int32_t depthSize;
        int32_t imageSize;

        const std::string file;             ///< 记录文件的路径
        FILE * fp;                          ///< C 风格的文件指针
        int32_t numFrames;          ///< 记录文件中帧的个数
        int width;                  ///< 输入图像的宽度
        int height;                 ///< 输入图像的高度
        int numPixels;              ///< 输入图像的像素数目

        JPEGLoader jpeg;
};

#endif /* LOGREADER_H_ */
