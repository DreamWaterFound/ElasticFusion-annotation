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

/* 记录文件的数据格式: 
 * 0x..00   int32_t     当前数据集中的帧数
 * 0x..04   int64_t     第0帧数据的时间戳
 * 0x..0C   int32_t         -深度图像数据长度 depthSize
 * 0x..10   int32_t         -彩色图像数据长度 imageSize
 * 0x..14   depthSize       -深度图像数据
 * ..       imageSize       -彩色图像数据(不一定有)
 * ..       int64_t     第1帧数据的时间戳
*/

// 原始记录文件读取器的构造函数
RawLogReader::RawLogReader(std::string file, bool flipColors)
    // 父类的构造函数
 : LogReader(file, flipColors)
{
    // 确保记录文件的确存在
    assert(pangolin::FileExists(file.c_str()));

    // 以只读的二进制方式打开这个文件
    fp = fopen(file.c_str(), "rb");

    // 当前帧的id (也就是技术)
    currentFrame = 0;

    // 读取记录文件中的帧的个数
    auto tmp = fread(&numFrames,sizeof(int32_t),1,fp);
    // 如果上面的操作过程没有读到有效的数据, 则 tmp =0
    assert(tmp);

    // 这个缓冲区中将保存从文件中直接读取的图像数据
    // 开辟缓冲深度图的 buffer, *2 是因为深度图每个像素是2个字节
    depthReadBuffer = new unsigned char[numPixels * 2];
    // 开辟缓冲彩色图的 buffer, *3 是因为rgb每个通道在一个像素中都要占用一个字节
    imageReadBuffer = new unsigned char[numPixels * 3];

    // 这儿的缓冲区则是保存经过解压缩和颜色通道顺序处理后的图像数据, 可以直接被后面的过程使用
    // 下面使用的 Resolution::getInstance().numPixels() 和上面的 numPixels 是完全一样的
    // 开辟解压缩深度图的缓冲区空间
    decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];
    // 开辟解压缩彩色图的缓冲区空间
    decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];
}

// 析构函数, 释放缓冲区内容, 关闭文件
RawLogReader::~RawLogReader()
{
    delete [] depthReadBuffer;
    delete [] imageReadBuffer;
    delete [] decompressionBufferDepth;
    delete [] decompressionBufferImage;

    fclose(fp);
}

// 设置文件指针返回到上一帧图像开始的位置
void RawLogReader::getBack()
{
    assert(filePointers.size() > 0);

    fseek(fp,                   // 要重新设置的文件指针
          filePointers.top(),   // offset, 就是上一帧数据相对于文件头部的偏移量
          SEEK_SET);            // form where

    // 因为已经回到了上一帧, 所以也要弹出曾经压入堆栈的数据
    filePointers.pop();

    // 获取真正的帧数据
    getCore();
}

// 获取下一帧图像数据, 保存到缓冲区中
void RawLogReader::getNext()
{
    // 保存当前文件指针相对于文件头部的偏移字节数
    filePointers.push(ftell(fp));
    getCore();
}

// 获取真正的帧数据到缓冲区中
void RawLogReader::getCore()
{
    // step 1 读取当前帧的时间戳
    auto tmp = fread(&timestamp,          // 保存位置
                     sizeof(int64_t),     // 读取长度
                     1,                   // 读取几个这样的对象
                     fp);                 // 当前的文件指针
    assert(tmp);

    // step 2 获取深度图像和彩色图像的数据长度
    tmp = fread(&depthSize,sizeof(int32_t),1,fp);
    assert(tmp);
    tmp = fread(&imageSize,sizeof(int32_t),1,fp);
    assert(tmp);

    // step 3 读取深度图像数据
    tmp = fread(depthReadBuffer,depthSize,1,fp);
    assert(tmp);

    // step 4 读取彩色图像数据
    if(imageSize > 0)
    {
        // ?
        // 这里之所以需要判断一下, 我猜是因为考虑到彩色图像和深度图像的同步问题, 会出现先存一张深度图像;
        // 但是如果这张图像对应的彩色图像先来, 就会没有存储, 对应到这里就需要判断是否是没有存对应的深度图像
        tmp = fread(imageReadBuffer,imageSize,1,fp);
        assert(tmp);
    }

    // step 5 解压缩深度图像
    // 如果读取到的深度图像的大小和构造本对象的时候的数据相同, 那么就直接复制
    // 为什么不直接在前面从文件中读取的过程中就进行检查呢? 我的猜想这样做的目的应该就是想让文件指针指向正确的位置
    if(depthSize == numPixels * 2)
    {
        memcpy(&decompressionBufferDepth[0], depthReadBuffer, numPixels * 2);
    }
    else
    {
        // 如果大小不相等, 说明保存的是压缩后的图像数据, 这里使用 zlib 库工具进行 in-memory (相对于解压/压缩文件来讲) 解压处理
        // ref: [https://blog.csdn.net/turingo/article/details/8148264]
        unsigned long decompLength = numPixels * 2;
        uncompress(&decompressionBufferDepth[0], (unsigned long *)&decompLength,    // 解压后的图像及其大小
                   (const Bytef *)depthReadBuffer, depthSize);                      // 待解压的图像及其大小
    }

    // step 6 解压缩彩色图像
    if(imageSize == numPixels * 3)
    {
        memcpy(&decompressionBufferImage[0], imageReadBuffer, numPixels * 3);
    }
    else if(imageSize > 0)
    {
        // 和深度图像不同的是这里直接使用 jpeg 方式解压
        jpeg.readData(imageReadBuffer, imageSize, (unsigned char *)&decompressionBufferImage[0]);
    }
    else
    {
        // 如果 imageSize = 0 说明对应的彩色图像并没有成功被读取
        memset(&decompressionBufferImage[0], 0, numPixels * 3);
    }

    depth = (unsigned short *)decompressionBufferDepth;
    rgb = (unsigned char *)&decompressionBufferImage[0];

    if(flipColors)
    {
        // 如要定义要翻转图像, 那么这里也是手动翻转 R 通道和 B 通道
        for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }

    // 当前帧计数(id)++
    currentFrame++;
}

// 快进到第 frame 帧
void RawLogReader::fastForward(int frame)
{
    // 当没有超过指定帧id frame 并还有图像数据的时候读取
    while(currentFrame < frame && hasMore())
    {
        // 记录当前帧相对于文件头部的偏移量
        filePointers.push(ftell(fp));

        // 时间戳信息
        auto tmp = fread(&timestamp,sizeof(int64_t),1,fp);
        assert(tmp);

        // 深度图像和彩色图像大小信息
        tmp = fread(&depthSize,sizeof(int32_t),1,fp);
        assert(tmp);
        tmp = fread(&imageSize,sizeof(int32_t),1,fp);
        assert(tmp);

        // 跳过深度图像和彩色图像的处理
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

// 获取记录文件中的帧的个数
int RawLogReader::getNumFrames()
{
    return numFrames;
}

// 判断记录文件中的内容是否已经读取完毕
bool RawLogReader::hasMore()
{
    return currentFrame + 1 < numFrames;
}

// 重新设置记录文件的读取
void RawLogReader::rewind()
{
    // 如果每一帧相对于文件头部的偏移量的栈不为空, 就删除
    if (filePointers.size() != 0)
    {
        // 神奇的删除方式...
        std::stack<int> empty;
        std::swap(empty, filePointers);
    }

    // 关闭之前已经打开过的记录文件并重新打开
    fclose(fp);
    fp = fopen(file.c_str(), "rb");

    // 读取记录文件中的帧数, 如果为0则触发断言错误
    auto tmp = fread(&numFrames,sizeof(int32_t),1,fp);
    assert(tmp);

    // 复位当前帧id
    currentFrame = 0;
}

// 判断是否需要倒带 -- 即记录文件倒序播放到头后, 再正序播放
bool RawLogReader::rewound()
{
    // 由于倒序播放过程中堆栈 filePointers 会一级级弹出, 所以当其为空的时候证明: 我们应该倒带了, 从倒序播放转变成为正序播放
    return filePointers.size() == 0;
}

// 获取记录文件路径
const std::string RawLogReader::getFile()
{
    return file;
}

// ? 自动曝光/白平衡参数设置? 对于记录文件无法设置, 空函数
void RawLogReader::setAuto(bool value)
{

}
