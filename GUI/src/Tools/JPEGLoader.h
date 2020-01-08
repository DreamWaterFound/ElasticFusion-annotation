/**
 * @file JPEGLoader.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief JPEG 编解码支持
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

#ifndef TOOLS_JPEGLOADER_H_
#define TOOLS_JPEGLOADER_H_

extern "C"
{
// jpeg 编解码支持
#include "jpeglib.h"
}

// C STL
#include <stdio.h>
#include <string>

/**
 * @brief 发生 JPEG 错误的时候的回调函数
 * @param[in] cinfo 解码器对象
 */
static void jpegFail(j_common_ptr cinfo)
{
    // 直接给出断言错误
    assert(false && "JPEG decoding error!");
}

/** @brief 空函数, 什么也不做 */
static void doNothing(j_decompress_ptr)
{

}

/** @brief JPEG 编解码支持 */
class JPEGLoader
{
    public:
        /** @brief 空构造函数 */
        JPEGLoader()
        {}

        /**
         * @brief 从JPEG编码压缩的图像中恢复出原始图像
         * @param[in]  src          从文件中读取的原始图像存放的缓冲区
         * @param[in]  numBytes     原始编码格式的图像长度
         * @param[out] data         解码之后存放图像的缓冲区
         */
        void readData(unsigned char * src, const int numBytes, unsigned char * data)
        {
            // step 0 准备数据
            jpeg_decompress_struct cinfo;           // IJG JPEG codec structure 解压缩器对象
            jpeg_error_mgr errorMgr;                // 错误发生的时候的管理器

            // step 1 错误对象设置
            errorMgr.error_exit = jpegFail;         // 设置发生解码错误的时候回调的函数
            cinfo.err = jpeg_std_error(&errorMgr);  // 设置错误管理器
            jpeg_create_decompress(&cinfo);         // 创建对象

            // step 2 数据源对象设置
            jpeg_source_mgr srcMgr;                 // 数据源管理器
            cinfo.src = &srcMgr;

            // Prepare for suspending reader
            srcMgr.init_source = doNothing;         // ? 不知道是干嘛的, 设置为了一个空函数; 感觉可能是这个库之前仅支持从文件中获取数据有关
            srcMgr.resync_to_restart = jpeg_resync_to_restart;  // 好像是处理数据源发生的时间不同步问题, 使用了 libJpeg 的默认处理函数
            srcMgr.term_source = doNothing;         // ? 同上
            srcMgr.next_input_byte = src;           // 设置需要解压缩图像缓冲区首指针
            srcMgr.bytes_in_buffer = numBytes;      // 设置需要解压缩图像缓冲区的长度

            // 启动解压过程, 查看数据流的内容, 从数据流的头部获取关于图像的大小/通道数等信息
            jpeg_read_header(&cinfo,                // 解压对象
                             TRUE);                 // 是否获取图像

            // 计算解压缩输出图像的维度
            jpeg_calc_output_dimensions(&cinfo);

            // 开始解压缩过程
            jpeg_start_decompress(&cinfo);

            // 获取输出图像的大小
            int width  = cinfo.output_width;
            int height = cinfo.output_height;

            // JSAMPARRAY 本质就是 uint8_t*
            // 由于 libJPEG 使用C编写, 考虑到动态内存的申请相对耗时, 并且容易忘记释放, libJPEG 中使用 "pool" 的概念来处理, 有点线程池的感觉
            // 说白了就是申请一大段内存, 然后重复使用. 下面函数参数中的 JPOOL_IMAGE 就表示一旦处理完之后就自动放弃对原 pool 中对应空间的占有
            JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)(         // 调用解压对象的 mem manger, 在 pool 中获取一段 buffer
                (j_common_ptr)&cinfo,                               // j_common_ptr 对象指针
                JPOOL_IMAGE,                                        // pool_id, lasts until done with image/datastream
                width * 4,                                          // samplesperrow, 一个像素占四个字节, RGB32格式, ref: [https://blog.csdn.net/byhook/article/details/84262330]
                1);                                                 // 一行图像大小就够了, 因为后面的 jpeg_read_scanlines() 函数只能一行行地读取计算结果

            // 将解码结果复制到输出缓冲区
            for(; height--; data += (width * 3))
            {
                // 从解压结果中获取一行
                jpeg_read_scanlines(    
                    &cinfo,             // 解压对象
                    buffer,             // 输出位置
                    1);                 // 读取一行解压图像

                // 直接读取得到的戒烟图像颜色通道是 BGR, RGB32 格式中是这样定义的, 所以接下来还要进行颜色通道的转换
                unsigned char * bgr = (unsigned char *)buffer[0];
                unsigned char * rgb = (unsigned char *)data;

                // BGR => RGB
                for(int i = 0; i < width; i++, bgr += 3, rgb += 3)
                {
                    unsigned char t0 = bgr[0], t1 = bgr[1], t2 = bgr[2];
                    rgb[2] = t0; rgb[1] = t1; rgb[0] = t2;
                }
            }

            // 解压缩过程结束, 释放对象
            jpeg_finish_decompress(&cinfo);
            jpeg_destroy_decompress(&cinfo);
        }
};


#endif /* TOOLS_JPEGLOADER_H_ */
