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

#include <stdio.h>
#include <stdlib.h>
#ifndef WIN32
#  include <poll.h>
#endif
#include <signal.h>
#include <chrono>
#include <thread>

#include <Utils/Parse.h>

#include "LogReader.h"
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
   * 
   * @param[in] file         记录文件, 这里实际上只用于构造父类对象, 并没有真正使用
   * @param[in] flipColors   是否左右翻转图像
   * @param[in] type         摄像头的类型
   */
		LiveLogReader(std::string file, bool flipColors, CameraType type);

		virtual ~LiveLogReader();

        void getNext();

        int getNumFrames();

        bool hasMore();

        bool rewound()
        {
            return false;
        }

        void rewind()
        {

        }

        void getBack()
        {

        }

        void fastForward(int frame)
        {

        }

        const std::string getFile();

        void setAuto(bool value);

    
		CameraInterface * cam;      ///< 保存摄像头的接口对象指针

	private:
		int64_t lastFrameTime;      /// 上一帧(结构体, 包含彩色图像和深度图像)的时间戳
		int lastGot;                /// ? 
};

#endif /* LIVELOGREADER_H_ */
