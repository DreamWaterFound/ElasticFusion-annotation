/**
 * @file OpenNI2Interface.h
 * @author guoqing (1337841346@qq.com)
 * @brief OpenNI的相机接口
 * @version 0.1
 * @date 2020-01-04
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

#ifndef OPENNI2INTERFACE_H_
#define OPENNI2INTERFACE_H_

// 和OpenNI相关的头文件
#include <OpenNI.h>
#include <PS1080.h>

// C++ STL
#include <string>
#include <iostream>
#include <algorithm>
#include <map>

// 增强的互斥锁类
#include "ThreadMutexObject.h"
// 相机接口虚基类
#include "CameraInterface.h"

/** @brief OpenNI 的相机接口 */
class OpenNI2Interface : public CameraInterface
{
    public:
        /**
         * @brief 构造函数
         * @param[in] inWidth 图像宽度
         * @param[in] inHeight 图像高度
         * @param[in] fps 打开的图像帧率
         */
        OpenNI2Interface(int inWidth = 640, int inHeight = 480, int fps = 30);
        /** @brief 析构函数 */
        virtual ~OpenNI2Interface();

        /// 图像的大小和帧率
        const int width, height, fps;

        /** @brief 在屏幕上输出当前设置的各种模式 */
        void printModes();

        /**
         * @brief 判断某一种视频格式是否存在
         * @param[in] x 图像宽度
         * @param[in] y 图像高度
         * @param[in] fps 图像帧率
         * @return 是否存在
         */
        bool findMode(int x, int y, int fps);

        /**
         * @brief 设置自动曝光
         * @param[in] value 是/否
         */
        virtual void setAutoExposure(bool value);

        /**
         * @brief 设置自动白平衡
         * @param[in] value 是/否
         */
        virtual void setAutoWhiteBalance(bool value);

        /**
         * @brief 获取当前是否设置了自动曝光
         * @return true 
         * @return false 
         */
        bool getAutoExposure();
        
        /**
         * @brief 获取当前是否设置了自动白平衡
         * 
         * @return true 
         * @return false 
         */
        bool getAutoWhiteBalance();

        /**
         * @brief 返回是否初始化成功
         * @return 是/否
         */
        virtual bool ok()
        {
            return initSuccessful;
        }

        /**
         * @brief 输出错误信息
         * @return std::string 文本形式的错误信息
         */
        virtual std::string error()
        {
            errorText.erase(std::remove_if(errorText.begin(), errorText.end(), &OpenNI2Interface::isTab), errorText.end());
            return errorText;
        }

        /** @brief 彩色图像到来的时候的回调函数对象, 基于OpenNI 提供的事件驱动机制 */
        class RGBCallback : public openni::VideoStream::NewFrameListener
        {
            public:
                /**
                 * @brief 构造函数, 初始化一些值
                 * @param[in] lastRgbTime 存储上一帧彩色图像时间戳的引用
                 * @param[in] latestRgbIndex 存储上一帧彩色图像id的引用
                 * @param[in] rgbBuffers 彩色图像缓冲区
                 */
                RGBCallback(int64_t & lastRgbTime,
                            ThreadMutexObject<int> & latestRgbIndex,
                            std::pair<uint8_t *, int64_t> * rgbBuffers)
                 : lastRgbTime(lastRgbTime),
                   latestRgbIndex(latestRgbIndex),
                   rgbBuffers(rgbBuffers)
                {}

                /** @brief 空析构函数 */
                virtual ~RGBCallback() {}

                /**
                 * @brief 当新帧到来的时候自动调用的函数, 实现将获取到的图像保存到缓冲区中并打时间戳的功能
                 * @param[in] stream 当前视频流
                 */
                void onNewFrame(openni::VideoStream& stream)
                {
                    stream.readFrame(&frame);

                    // 根据当前系统时间生成时间戳
                    lastRgbTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::system_clock::now().time_since_epoch()).count();

                    // 将当前的图像存入到缓冲区指定位置, 并打上时间戳
                    int bufferIndex = (latestRgbIndex.getValue() + 1) % numBuffers;
                    memcpy(rgbBuffers[bufferIndex].first, frame.getData(), frame.getWidth() * frame.getHeight() * 3);
                    rgbBuffers[bufferIndex].second = lastRgbTime;
                    latestRgbIndex++;
                }

            private:
                openni::VideoFrameRef frame;                ///< 缓存当前的 OpenNI 帧对象
                int64_t & lastRgbTime;                      ///< 上一帧彩色图像时间戳的引用
                ThreadMutexObject<int> & latestRgbIndex;    ///< 上一帧彩色图像id的引用
                std::pair<uint8_t *, int64_t> * rgbBuffers; ///< 彩色图像缓冲区
        };

        /** @brief 深度图像到来的时候的回调函数对象, 基于OpenNI 提供的事件驱动机制 */
        class DepthCallback : public openni::VideoStream::NewFrameListener
        {
            public:
                /**
                 * @brief 构造函数, 初始化一些值
                 * @param[in] lastDepthTime         上一帧深度图像时间戳的引用
                 * @param[in] latestDepthIndex      上一帧深度图像id的引用
                 * @param[in] latestRgbIndex        上一帧彩色图像id的引用
                 * @param[in] rgbBuffers            彩色图像缓冲区
                 * @param[in] frameBuffers          帧缓冲区
                 */
                DepthCallback(int64_t & lastDepthTime,
                              ThreadMutexObject<int> & latestDepthIndex,
                              ThreadMutexObject<int> & latestRgbIndex,
                              std::pair<uint8_t *, int64_t> * rgbBuffers,
                              std::pair<std::pair<uint8_t *, uint8_t *>, int64_t> * frameBuffers)
                 : lastDepthTime(lastDepthTime),
                   latestDepthIndex(latestDepthIndex),
                   latestRgbIndex(latestRgbIndex),
                   rgbBuffers(rgbBuffers),
                   frameBuffers(frameBuffers)
                {}

                /** @brief 空析构函数 */
                virtual ~DepthCallback() {}

                /**
                 * @brief 当新的深度图像到来时的回调
                 * @param[in] stream 外部函数调用的时候给的图像数据流
                 */
                void onNewFrame(openni::VideoStream& stream)
                {
                    stream.readFrame(&frame);

                    // 生成深度图像的时间戳
                    lastDepthTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::system_clock::now().time_since_epoch()).count();

                    // 将新深度图像保存到帧的缓冲区中的对应位置
                    int bufferIndex = (latestDepthIndex.getValue() + 1) % numBuffers;
                    memcpy(frameBuffers[bufferIndex].first.first, frame.getData(), frame.getWidth() * frame.getHeight() * 2);
                    frameBuffers[bufferIndex].second = lastDepthTime;

                    // 下面是准备找彩色图像和深度图像的关联关系, 首先获取最近的彩色图像id
                    int lastImageVal = latestRgbIndex.getValue();
                    // 如果为-1说明彩色图像没有到, 那么就先等等(相当于放弃了当前帧的深度图像)
                    if(lastImageVal == -1)
                    {
                        return;
                    }

                    // 如果找到了对应的彩色图像, 那么就去彩色图像缓冲区中索引, 将对应的彩色图像保存到帧缓冲区中的对应位置
                    lastImageVal %= numBuffers;
                    memcpy(frameBuffers[bufferIndex].first.second, rgbBuffers[lastImageVal].first, frame.getWidth() * frame.getHeight() * 3);
                    latestDepthIndex++;
                }

            private:
                openni::VideoFrameRef frame;                    ///< OpenNI的帧缓存
                int64_t & lastDepthTime;                        ///< 上一帧深度图像时间戳的引用
                ThreadMutexObject<int> & latestDepthIndex;      ///< 上一帧深度图像id的引用
                ThreadMutexObject<int> & latestRgbIndex;        ///< 上一帧彩色图像id的引用

                std::pair<uint8_t *, int64_t> * rgbBuffers;     ///< 彩色图像缓冲区
                std::pair<
                    std::pair<uint8_t *, uint8_t *>, 
                    int64_t> * frameBuffers;                    ///< 帧的缓冲区
        };

    private:
        openni::Device device;                                  ///< OpenNI2 设备对象

        
        openni::VideoStream depthStream;                        ///< OpenNI2 深度图像流
        openni::VideoStream rgbStream;                          ///< OpenNI2 彩色图像流

        // Map for formats from OpenNI2
        std::map<int, std::string> formatMap;                   ///< 保存不同流的类型和对应的字符串描述, 建立了id和字符串描述的关系

        int64_t lastRgbTime;                                    ///< 上一帧的彩色图像时间戳
        int64_t lastDepthTime;                                  ///< 上一帧的深度图像时间戳

        ThreadMutexObject<int> latestRgbIndex;                  ///< 有线程锁保护的, 记录最近彩色图像的id
        std::pair<uint8_t *, int64_t> rgbBuffers[numBuffers];   ///< 彩色图像缓冲区, 第一个元素是缓冲区指针, 第二个元素是时间戳

        RGBCallback * rgbCallback;                              ///< 彩色帧数据监听器
        DepthCallback * depthCallback;                          ///< 深度数据监听器

        
        bool initSuccessful;                                    ///< 标记是否初始化成功了
        std::string errorText;                                  ///< 缓存操作 OpenNI 接口过程中的错误提示信息

        /**
         * @brief For removing tabs from OpenNI's error messages
         * @note 从错误信息的文本描述中删除 tab 符, 其实就是判断传入的某个参数是否是tab符
         * @param[in] c 要判断的字符
         * @return true 
         * @return false 
         */
        static bool isTab(char c)
        {
            switch(c)
            {
                case '\t':
                    return true;
                default:
                    return false;
            }
        }
};

#endif /* OPENNI2INTERFACE_H_ */
