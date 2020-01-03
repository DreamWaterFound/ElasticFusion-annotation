/**
 * @file RawLogReader.cpp
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

#include "RawLogReader.h"

// 原始记录文件读取器的构造函数
RawLogReader::RawLogReader(std::string file, bool flipColors)
    // 父类的构造函数
 : LogReader(file, flipColors)
{
    // 确保记录文件的确存在
    assert(pangolin::FileExists(file.c_str()));

    // 以只读的二进制方式打开这个文件
    fp = fopen(file.c_str(), "rb");

    // ? 当前帧的偏移量?
    currentFrame = 0;

    // 读取记录文件中的帧的个数
    auto tmp = fread(&numFrames,sizeof(int32_t),1,fp);
    // 如果上面的操作过程没有读到有效的数据, 则 tmp =0
    assert(tmp);

    // 开辟缓冲深度图的 buffer, *2 是因为深度图每个像素是2个字节
    depthReadBuffer = new unsigned char[numPixels * 2];
    // 开辟缓冲彩色图的 buffer, *3 是因为rgb每个通道在一个像素中都要占用一个字节
    imageReadBuffer = new unsigned char[numPixels * 3];

    // 下面使用的 Resolution::getInstance().numPixels() 和上面的 numPixels 是完全一样的
    // 开辟解压缩深度图的缓冲区空间
    decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];
    // 开辟解压缩彩色图的缓冲区空间
    decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];

    // ? 上面的这两个缓冲区有什么不同?
}

RawLogReader::~RawLogReader()
{
    delete [] depthReadBuffer;
    delete [] imageReadBuffer;
    delete [] decompressionBufferDepth;
    delete [] decompressionBufferImage;

    fclose(fp);
}

void RawLogReader::getBack()
{
    assert(filePointers.size() > 0);

    fseek(fp, filePointers.top(), SEEK_SET);

    filePointers.pop();

    getCore();
}

void RawLogReader::getNext()
{
    filePointers.push(ftell(fp));

    getCore();
}

void RawLogReader::getCore()
{
    auto tmp = fread(&timestamp,sizeof(int64_t),1,fp);
    assert(tmp);

    tmp = fread(&depthSize,sizeof(int32_t),1,fp);
    assert(tmp);
    tmp = fread(&imageSize,sizeof(int32_t),1,fp);
    assert(tmp);

    tmp = fread(depthReadBuffer,depthSize,1,fp);
    assert(tmp);

    if(imageSize > 0)
    {
        tmp = fread(imageReadBuffer,imageSize,1,fp);
        assert(tmp);
    }

    if(depthSize == numPixels * 2)
    {
        memcpy(&decompressionBufferDepth[0], depthReadBuffer, numPixels * 2);
    }
    else
    {
        unsigned long decompLength = numPixels * 2;
        uncompress(&decompressionBufferDepth[0], (unsigned long *)&decompLength, (const Bytef *)depthReadBuffer, depthSize);
    }

    if(imageSize == numPixels * 3)
    {
        memcpy(&decompressionBufferImage[0], imageReadBuffer, numPixels * 3);
    }
    else if(imageSize > 0)
    {
        jpeg.readData(imageReadBuffer, imageSize, (unsigned char *)&decompressionBufferImage[0]);
    }
    else
    {
        memset(&decompressionBufferImage[0], 0, numPixels * 3);
    }

    depth = (unsigned short *)decompressionBufferDepth;
    rgb = (unsigned char *)&decompressionBufferImage[0];

    if(flipColors)
    {
        for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }

    currentFrame++;
}

void RawLogReader::fastForward(int frame)
{
    while(currentFrame < frame && hasMore())
    {
        filePointers.push(ftell(fp));

        auto tmp = fread(&timestamp,sizeof(int64_t),1,fp);
        assert(tmp);

        tmp = fread(&depthSize,sizeof(int32_t),1,fp);
        assert(tmp);
        tmp = fread(&imageSize,sizeof(int32_t),1,fp);
        assert(tmp);

        tmp = fread(depthReadBuffer,depthSize,1,fp);
        assert(tmp);

        if(imageSize > 0)
        {
            tmp = fread(imageReadBuffer,imageSize,1,fp);
            assert(tmp);
        }

        currentFrame++;
    }
}

int RawLogReader::getNumFrames()
{
    return numFrames;
}

bool RawLogReader::hasMore()
{
    return currentFrame + 1 < numFrames;
}


void RawLogReader::rewind()
{
    if (filePointers.size() != 0)
    {
        std::stack<int> empty;
        std::swap(empty, filePointers);
    }

    fclose(fp);
    fp = fopen(file.c_str(), "rb");

    auto tmp = fread(&numFrames,sizeof(int32_t),1,fp);
    assert(tmp);

    currentFrame = 0;
}

bool RawLogReader::rewound()
{
    return filePointers.size() == 0;
}

const std::string RawLogReader::getFile()
{
    return file;
}

void RawLogReader::setAuto(bool value)
{

}
