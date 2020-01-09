/**
 * @file LiveLogReader.h
 * @author guoqing (1337841346@qq.com)
 * @brief 读取实时传感器数据
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


#ifndef LIVELOGREADER_H_
#define LIVELOGREADER_H_

// C STL
#include <stdio.h>
#include <stdlib.h>
#ifndef WIN32
#  include <poll.h>
#endif
// Linux
#include <signal.h>
// C++ STL
#include <chrono>
#include <thread>

// 命令行解析器
#include <Utils/Parse.h>

// 父类, 记录文件加载器
#include "LogReader.h"
// 相机接口
#include "CameraInterface.h"

/** @brief 读取实时数据的类 */
class LiveLogReader : public LogReader
{
	public:
    /**
     * @brief 摄像头类型就两种, OpenNI支持的系列, 以及Realsense系列
     * 
     */
    enum CameraType
    {
      OpenNI2,RealSense
    };

  /**
   * @brief 构造函数
   * @param[in] file         记录文件, 这里实际上只用于构造父类对象, 并没有真正使用
   * @param[in] flipColors   是否左右翻转图像
   * @param[in] type         摄像头的类型
   */
		LiveLogReader(std::string file, bool flipColors, CameraType type);

    /** @brief 析构函数, 释放缓冲区, 析构相机对象 */
		virtual ~LiveLogReader();

        /** @brief 获取摄像头的下一帧图像到 LogReader 的缓冲区中 */
        void getNext();

        /**
         * @brief 获取可以从相机读取的帧数的最大值
         * @return int 帧数最大值
         */
        int getNumFrames();

        /** @brief 对于实时的摄像头, 数据是永远都会有的, 一直返回 true */
        bool hasMore();

        /**
         * @brief 是否需要"倒带"读取, 从逆序读取转换成为正序读取
         * @note 由于 live camera 不存在这个问题, 所以一直范围 false
         * @return false 
         */
        bool rewound()
        {
            return false;
        }

        /** @brief 如果是跑实际的摄像头, 就没有必要重新读取记录文件了, 这里设置为空操作 */
        void rewind(){ }

        /** @brief 对于实际摄像头没有重新读取的概念, 所以这个倒序读取图像的函数并不实现 */
        void getBack(){ }

        /**
         * @brief 本来是在记录文件中快速确定每一帧相对于文件开头的位置的, 但是对于 live camera 没有什么用, 函数本身并不实现
         * @param[in] frame 执行处理的最大帧
         */
        void fastForward(int frame){ }

        /**
         * @brief 获取记录文件路径
         * @return const std::string 不过对于实时摄像头而言, 返回的是当前的基路径+"live"
         */
        const std::string getFile();

        /**
         * @brief 设置自动曝光和自动白平衡
         * @param[in] value 是否
         */
        void setAuto(bool value);

    
		CameraInterface * cam;      ///< 保存摄像头的接口对象指针

	private:
		int64_t lastFrameTime;      /// 上一帧(结构体, 包含彩色图像和深度图像)的时间戳
		int lastGot;                /// 上一帧深度图像在图像缓冲区的id
};

#endif /* LIVELOGREADER_H_ */
