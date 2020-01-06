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


#include "LiveLogReader.h"

#include "OpenNI2Interface.h"
#include "RealSenseInterface.h"

// 构造函数
LiveLogReader::LiveLogReader(
            std::string file,   // ? 记录文件的路径, 这里没有使用到只是用于构造父类对象
            bool flipColors,    // 是否左右翻转图像
            CameraType type)    // 摄像头类型
 : LogReader(file, flipColors),
   lastFrameTime(-1),           // ?
   lastGot(-1)                  // ?
{
    // step 0 创建相应类型的相机接口
    std::cout << "Creating live capture... "; std::cout.flush();

    // HACK
     openni::Version version = openni::OpenNI::getVersion();
		std::cout << version.minor << "."
				<< version.major << "."
				<< version.maintenance << "."
				<< version.build 
				<< std::endl;

    if(type == CameraType::OpenNI2)
    {
      std::cout<<"type == CameraType::OpenNI2"<<std::endl;
      // HERE
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
      std::cout<<"type not matched."<<std::endl;
      cam = nullptr;
    }

	decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];

	decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];

    if(!cam || !cam->ok())
    {
        std::cout << "failed!" << std::endl;
        std::cout << cam->error();
    }
    else
    {
        std::cout << "success!" << std::endl;

        std::cout << "Waiting for first frame"; std::cout.flush();

        int lastDepth = cam->latestDepthIndex.getValue();

        do
        {
          std::this_thread::sleep_for(std::chrono::microseconds(33333));
            std::cout << "."; std::cout.flush();
            lastDepth = cam->latestDepthIndex.getValue();
        } while(lastDepth == -1);

        std::cout << " got it!" << std::endl;
    }
}

LiveLogReader::~LiveLogReader()
{
    delete [] decompressionBufferDepth;

    delete [] decompressionBufferImage;

	delete cam;
}

void LiveLogReader::getNext()
{
    int lastDepth = cam->latestDepthIndex.getValue();

    assert(lastDepth != -1);

    int bufferIndex = lastDepth % CameraInterface::numBuffers;

    if(bufferIndex == lastGot)
    {
        return;
    }

    if(lastFrameTime == cam->frameBuffers[bufferIndex].second)
    {
        return;
    }

    memcpy(&decompressionBufferDepth[0], cam->frameBuffers[bufferIndex].first.first, Resolution::getInstance().numPixels() * 2);
    memcpy(&decompressionBufferImage[0], cam->frameBuffers[bufferIndex].first.second,Resolution::getInstance().numPixels() * 3);

    lastFrameTime = cam->frameBuffers[bufferIndex].second;

    timestamp = lastFrameTime;

    rgb = (unsigned char *)&decompressionBufferImage[0];
    depth = (unsigned short *)&decompressionBufferDepth[0];

    imageReadBuffer = 0;
    depthReadBuffer = 0;

    imageSize = Resolution::getInstance().numPixels() * 3;
    depthSize = Resolution::getInstance().numPixels() * 2;

    if(flipColors)
    {
        for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }
}

const std::string LiveLogReader::getFile()
{
    return Parse::get().baseDir().append("live");
}

int LiveLogReader::getNumFrames()
{
    return std::numeric_limits<int>::max();
}

bool LiveLogReader::hasMore()
{
    return true;
}

void LiveLogReader::setAuto(bool value)
{
    cam->setAutoExposure(value);
    cam->setAutoWhiteBalance(value);
}
