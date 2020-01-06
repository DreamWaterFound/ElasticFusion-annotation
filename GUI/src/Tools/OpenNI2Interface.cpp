/**
 * @file OpenNI2Interface.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief OpenNI 的相机接口
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

#include "OpenNI2Interface.h"

OpenNI2Interface::OpenNI2Interface(int inWidth, int inHeight, int fps)
 : width(inWidth),
   height(inHeight),
   fps(fps),
   initSuccessful(true)
{
    // step 0 Setup
    // 用于保存后面各个 OpenNI 操作的状态
    openni::Status rc = openni::STATUS_OK;

    const char * deviceURI = openni::ANY_DEVICE;

    rc = openni::OpenNI::initialize();

    std::string errorString(openni::OpenNI::getExtendedError());

    if(errorString.length() > 0)
    {
        // 真的出现了错误
        errorText.append(errorString);
        initSuccessful = false;
    }
    else
    {
        // 上述操作一切顺利
        // step 1 打开设备
        rc = device.open(deviceURI);
        if (rc != openni::STATUS_OK)
        {
            // 打开失败
            errorText.append(openni::OpenNI::getExtendedError());
            openni::OpenNI::shutdown();
            initSuccessful = false;
        }
        else
        {
            // step 2.1 设置深度图像流和彩色图像流的属性
            openni::VideoMode depthMode;
            depthMode.setFps(fps);
            depthMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
            depthMode.setResolution(width, height);

            openni::VideoMode colorMode;
            colorMode.setFps(fps);
            colorMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
            colorMode.setResolution(width, height);

            // step 2.2 打开深度流
            rc = depthStream.create(device, openni::SENSOR_DEPTH);
            if (rc == openni::STATUS_OK)
            {
                depthStream.setVideoMode(depthMode);
                rc = depthStream.start();
                if (rc != openni::STATUS_OK)
                {
                    errorText.append(openni::OpenNI::getExtendedError());
                    depthStream.destroy();
                    initSuccessful = false;
                }
            }
            else
            {
                errorText.append(openni::OpenNI::getExtendedError());
                initSuccessful = false;
            }

            // step 2.3 打开视频流
            rc = rgbStream.create(device, openni::SENSOR_COLOR);
            if (rc == openni::STATUS_OK)
            {
                rgbStream.setVideoMode(colorMode);
                rc = rgbStream.start();
                if (rc != openni::STATUS_OK)
                {
                    errorText.append(openni::OpenNI::getExtendedError());
                    rgbStream.destroy();
                    initSuccessful = false;
                }
            }
            else
            {
                errorText.append(openni::OpenNI::getExtendedError());
                initSuccessful = false;
            }

            if (!depthStream.isValid() || !rgbStream.isValid())
            {
                errorText.append(openni::OpenNI::getExtendedError());
                openni::OpenNI::shutdown();
                initSuccessful = false;
            }

            // step 2.4 如果上述的各个步骤没有出现明显的问题, 那么就初始化缓冲区, 设置监听器, 并且进行基本的曝光白平衡设置
            if(initSuccessful)
            {
                // For printing out - 建立了流的数据类型和其字符串描述的关系
                formatMap[openni::PIXEL_FORMAT_DEPTH_1_MM] = "1mm";
                formatMap[openni::PIXEL_FORMAT_DEPTH_100_UM] = "100um";
                formatMap[openni::PIXEL_FORMAT_SHIFT_9_2] = "Shift 9 2";
                formatMap[openni::PIXEL_FORMAT_SHIFT_9_3] = "Shift 9 3";

                formatMap[openni::PIXEL_FORMAT_RGB888] = "RGB888";
                formatMap[openni::PIXEL_FORMAT_YUV422] = "YUV422";
                formatMap[openni::PIXEL_FORMAT_GRAY8] = "GRAY8";
                formatMap[openni::PIXEL_FORMAT_GRAY16] = "GRAY16";
                formatMap[openni::PIXEL_FORMAT_JPEG] = "JPEG";

                // 确认当前设置的数据流格式的确存在; 后面的字符串目测是在发生断言错误的时候用于提示的
                assert(findMode(width, height, fps) && "Sorry, mode not supported!");

                // 初始化彩色图和深度图的id, 目前还没有接收到任何图像所以id均设置成为-1
                latestDepthIndex.assign(-1);
                latestRgbIndex.assign(-1);

                // 初始化彩色图像缓冲区
                for(int i = 0; i < numBuffers; i++)
                {
                    // calloc 和 malloc 除了参数区别外, 前者还能够初始化分配的内存空间
                    uint8_t * newImage = (uint8_t *)calloc(width * height * 3, sizeof(uint8_t));
                    // 后面的 int64_t 表示的是时间戳(图像到达时的系统的时间戳)
                    rgbBuffers[i] = std::pair<uint8_t *, int64_t>(newImage, 0);
                }

                // 初始化帧图像缓冲区
                for(int i = 0; i < numBuffers; i++)
                {
                    // 深度图像
                    uint8_t * newDepth = (uint8_t *)calloc(width * height * 2, sizeof(uint8_t));
                    // 彩色图像
                    uint8_t * newImage = (uint8_t *)calloc(width * height * 3, sizeof(uint8_t));
                    frameBuffers[i] = std::pair<std::pair<uint8_t *, uint8_t *>, int64_t>(std::pair<uint8_t *, uint8_t *>(newDepth, newImage), 0);
                }

                // 设置回调函数,本质上是监听器, 基于 OpenNI 提供的事件机制
                rgbCallback = new RGBCallback(lastRgbTime,
                                              latestRgbIndex,
                                              rgbBuffers);

                // 这个是深度图像的回调, 本质上是监听器, 但是会直接生成帧数据(深度图+彩色图+深度图时间戳)
                depthCallback = new DepthCallback(lastDepthTime,
                                                  latestDepthIndex,
                                                  latestRgbIndex,
                                                  rgbBuffers,
                                                  frameBuffers);

                // 不要翻转图像
                depthStream.setMirroringEnabled(false);
                rgbStream.setMirroringEnabled(false);

                // 设置深度图和彩色图的配准方式, 
                device.setDepthColorSyncEnabled(true);
                device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

                // 使能自动曝光和白平衡(貌似对于Kinect1来说也没法关闭啊)
                setAutoExposure(true);
                setAutoWhiteBalance(true);

                // 添加监听器
                rgbStream.addNewFrameListener(rgbCallback);
                depthStream.addNewFrameListener(depthCallback);
            }
        }
    }
}

OpenNI2Interface::~OpenNI2Interface()
{
    if(initSuccessful)
    {
        rgbStream.removeNewFrameListener(rgbCallback);
        depthStream.removeNewFrameListener(depthCallback);

        depthStream.stop();
        rgbStream.stop();
        depthStream.destroy();
        rgbStream.destroy();
        device.close();
        openni::OpenNI::shutdown();

        for(int i = 0; i < numBuffers; i++)
        {
            free(rgbBuffers[i].first);
        }

        for(int i = 0; i < numBuffers; i++)
        {
            free(frameBuffers[i].first.first);
            free(frameBuffers[i].first.second);
        }

        delete rgbCallback;
        delete depthCallback;
    }
}

// 确认某一种视频格式是否存在
bool OpenNI2Interface::findMode(int x, int y, int fps)
{
    // step 1.1 获取当前 OpenNI 支持的深度数据流格式
    const openni::Array<openni::VideoMode> & depthModes = depthStream.getSensorInfo().getSupportedVideoModes();
    // 是否找到对应的视频格式
    bool found = false;

    // step 1.2 遍历, 如果有匹配到的就找到了
    for(int i = 0; i < depthModes.getSize(); i++)
    {
        if(depthModes[i].getResolutionX() == x &&
           depthModes[i].getResolutionY() == y &&
           depthModes[i].getFps() == fps)
        {
            found = true;
            break;
        }
    }

    if(!found)
    {
        return false;
    }

    found = false;

    // step 2 对于彩色数据流也进行相同的操作
    const openni::Array<openni::VideoMode> & rgbModes = rgbStream.getSensorInfo().getSupportedVideoModes();

    for(int i = 0; i < rgbModes.getSize(); i++)
    {
        if(rgbModes[i].getResolutionX() == x &&
           rgbModes[i].getResolutionY() == y &&
           rgbModes[i].getFps() == fps)
        {
            found = true;
            break;
        }
    }

    return found;
}

void OpenNI2Interface::printModes()
{
    const openni::Array<openni::VideoMode> & depthModes = depthStream.getSensorInfo().getSupportedVideoModes();

    openni::VideoMode currentDepth = depthStream.getVideoMode();

    std::cout << "Depth Modes: (" << currentDepth.getResolutionX() <<
                                     "x" <<
                                     currentDepth.getResolutionY() <<
                                     " @ " <<
                                     currentDepth.getFps() <<
                                     "fps " <<
                                     formatMap[currentDepth.getPixelFormat()] << ")" << std::endl;

    for(int i = 0; i < depthModes.getSize(); i++)
    {
        std::cout << depthModes[i].getResolutionX() <<
                     "x" <<
                     depthModes[i].getResolutionY() <<
                     " @ " <<
                     depthModes[i].getFps() <<
                     "fps " <<
                     formatMap[depthModes[i].getPixelFormat()] << std::endl;
    }

    const openni::Array<openni::VideoMode> & rgbModes = rgbStream.getSensorInfo().getSupportedVideoModes();

    openni::VideoMode currentRGB = depthStream.getVideoMode();

    std::cout << "RGB Modes: (" << currentRGB.getResolutionX() <<
                                   "x" <<
                                   currentRGB.getResolutionY() <<
                                   " @ " <<
                                   currentRGB.getFps() <<
                                   "fps " <<
                                   formatMap[currentRGB.getPixelFormat()] << ")" << std::endl;

    for(int i = 0; i < rgbModes.getSize(); i++)
    {
        std::cout << rgbModes[i].getResolutionX() <<
                     "x" <<
                     rgbModes[i].getResolutionY() <<
                     " @ " <<
                     rgbModes[i].getFps() <<
                     "fps " <<
                     formatMap[rgbModes[i].getPixelFormat()] << std::endl;
    }
}

// 设置自动曝光
void OpenNI2Interface::setAutoExposure(bool value)
{
    rgbStream.getCameraSettings()->setAutoExposureEnabled(value);
}

// 设置自动白平衡
void OpenNI2Interface::setAutoWhiteBalance(bool value)
{
    rgbStream.getCameraSettings()->setAutoWhiteBalanceEnabled(value);
}

bool OpenNI2Interface::getAutoExposure()
{
    return rgbStream.getCameraSettings()->getAutoExposureEnabled();
}

bool OpenNI2Interface::getAutoWhiteBalance()
{
    return rgbStream.getCameraSettings()->getAutoWhiteBalanceEnabled();
}
