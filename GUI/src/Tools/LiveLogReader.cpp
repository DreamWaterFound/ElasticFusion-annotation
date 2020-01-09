/**
 * @file LiveLogReader.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 获取实时的传感器数据
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

// 当前类定义
#include "LiveLogReader.h"

// 相关驱动的接口
#include "OpenNI2Interface.h"
#include "RealSenseInterface.h"

// 构造函数
LiveLogReader::LiveLogReader(
            std::string file,   // 记录文件的路径, 这里没有使用到只是用于构造父类对象
            bool flipColors,    // 是否左右翻转图像
            CameraType type)    // 摄像头类型
 : LogReader(file, flipColors),
   lastFrameTime(-1),           // 初始化上一帧对象的时间戳
   lastGot(-1)                  // 上一帧深度图像在图像缓冲区的id
{
    // step 0 创建相应类型的相机接口
    std::cout << "Creating live capture... "; std::cout.flush();

    // HACK 输出 OpenNI 版本号
    //  openni::Version version = openni::OpenNI::getVersion();
	// 	std::cout << version.minor << "."
	// 			<< version.major << "."
	// 			<< version.maintenance << "."
	// 			<< version.build 
	// 			<< std::endl;

    if(type == CameraType::OpenNI2)
    {
      std::cout<<"type == CameraType::OpenNI2"<<std::endl;
      // 生成 OpenNI 相机接口对象, 参数是图像的大小
      cam = new OpenNI2Interface(Resolution::getInstance().width(),Resolution::getInstance().height());
    }
    else if(type == CameraType::RealSense)
    {
      // 由于用不到, 暂时不管这里面的具体实现
      std::cout<<"type == CameraType::RealSense"<<std::endl;
      cam = new RealSenseInterface(Resolution::getInstance().width(), Resolution::getInstance().height());
    }
    else
    {
      // 没有指定, 那我就报错
      std::cout<<"type not matched."<<std::endl;
      cam = nullptr;
    }

    // step 1 分配解码的深度图像和彩色图像缓冲区
	decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];
	decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];

    // step 2 检查相机状态, 等待第一帧数据出现
    if(!cam || !cam->ok())
    {
        // 相机不 ok
        std::cout << "failed!" << std::endl;
        std::cout << cam->error();
    }
    else
    {
        // 相机正常工作
        std::cout << "success!" << std::endl;
        std::cout << "Waiting for first frame"; std::cout.flush();

        // 获取初始深度图像id
        int lastDepth = cam->latestDepthIndex.getValue();

        do
        {
            // 等, 直到第一张图像出来
            std::this_thread::sleep_for(std::chrono::microseconds(33333));
            std::cout << "."; std::cout.flush();
            lastDepth = cam->latestDepthIndex.getValue();
        } while(lastDepth == -1);

        std::cout << " got it!" << std::endl;
    }

}

// 析构函数, 释放缓冲区, 析构相机对象
LiveLogReader::~LiveLogReader()
{
    delete [] decompressionBufferDepth;

    delete [] decompressionBufferImage;

	delete cam;
}

// 获取摄像头的下一帧图像到 LogReader 的缓冲区中
void LiveLogReader::getNext()
{
    // 最新帧深度图像的id
    int lastDepth = cam->latestDepthIndex.getValue();

    // 说明摄像头就没有正确地打开过
    assert(lastDepth != -1);

    // 确定这个图像缓冲到的位置(在缓冲区的下标)
    int bufferIndex = lastDepth % CameraInterface::numBuffers;

    //  如果最新帧图像的时间戳和上一帧图像相同, 或者在缓冲区的下标相同, 说明新的图像还没有来, 我们现在跳过这个部分的处理
    if(bufferIndex == lastGot)    { return; }
    if(lastFrameTime == cam->frameBuffers[bufferIndex].second)    { return; }

    // 现在确定是真的来了新的图像, 拷贝到我们的缓冲区中
    memcpy(&decompressionBufferDepth[0], cam->frameBuffers[bufferIndex].first.first, Resolution::getInstance().numPixels() * 2);
    memcpy(&decompressionBufferImage[0], cam->frameBuffers[bufferIndex].first.second,Resolution::getInstance().numPixels() * 3);

    // 更新时间戳和数据缓冲区头指针
    lastFrameTime = cam->frameBuffers[bufferIndex].second;
    timestamp = lastFrameTime;
    rgb = (unsigned char *)&decompressionBufferImage[0];
    depth = (unsigned short *)&decompressionBufferDepth[0];
    // 对于live camera没有从文件中读取的这个过程, 这两个缓冲区用不到
    // ! 但是设置成为 nullptr 会更好
    imageReadBuffer = 0;
    depthReadBuffer = 0;
    // 计算深度图像和彩色图像的大小
    imageSize = Resolution::getInstance().numPixels() * 3;
    depthSize = Resolution::getInstance().numPixels() * 2;
    // 看是否需要翻转图像的 R 通道和 B 通道
    if(flipColors)
    {
        for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }
}

// 获取记录文件路径, 但是对于实时运行的摄像头而言, 这里就直接返回:
// "当前路径+live"
const std::string LiveLogReader::getFile()
{
    return Parse::get().baseDir().append("live");
}

// 获取可以从相机读取的帧数的最大值
int LiveLogReader::getNumFrames()
{
    // ummm... 由于ElasticFusion 中使用 int 对象表示其id, 所以能够读取的帧数最大值也就是 int 型所能够表示的数据范围
    return std::numeric_limits<int>::max();
}

// 对于实时的摄像头, 数据是永远都会有的, 一直返回 true
bool LiveLogReader::hasMore()
{
    return true;
}
// 设置自动曝光和自动白平衡
void LiveLogReader::setAuto(bool value)
{
    cam->setAutoExposure(value);
    cam->setAutoWhiteBalance(value);
}
