/**
 * @file MainController.h
 * @author guoqing (1337841346@qq.com)
 * @brief 定义 MainController 类
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

// ElasticFusion 对象
#include <ElasticFusion.h>
// 命令行参数解析
#include <Utils/Parse.h>

// GUI 界面
#include "Tools/GUI.h"
// 加载真值比较用的
#include "Tools/GroundTruthOdometry.h"
// 数据源Base
#include "Tools/RawLogReader.h"
// 实时摄像头数据
#include "Tools/LiveLogReader.h"

#ifndef MAINCONTROLLER_H_
#define MAINCONTROLLER_H_

// GUI 接口的实现的类
class MainController
{
    public:
        /**
         * @brief 构造函数
         * @param[in] argc 命令行参数个数
         * @param[in] argv 命令行参数内容
         */
        MainController(int argc, char * argv[]);

        /** @brief 析构函数 */
        virtual ~MainController();

        /** @brief 运行 */
        void launch();

    private:
        /* @brief 主循环, 实现给 ElasticFusion 喂图像的工作 */
        void run();

        /**
         * @brief 加载指定文件中的相机内参信息
         * @param[in] filename 文件名
         */
        void loadCalibration(const std::string & filename);

        bool good;                                      ///< 当前是否已经正确地初始化数据源了
        ElasticFusion * eFusion;                        ///< ElasticFusion Core 对象指针
        GUI * gui;                                      ///< GUI 窗口对象
        GroundTruthOdometry * groundTruthOdometry;      ///< 轨迹真值文件对象句柄
        LogReader * logReader;                          ///< 记录文件读取器的句柄, 根据情况也会是实时相机的接口

        bool iclnuim;                                   ///< 是否使用 ICL-NUIM 数据集
        std::string logFile;                            ///< 命令行中指定的记录文件路径
        std::string poseFile;                           ///< 命令行参数中指定的真值文件

        float confidence,                               ///< Surfel confidence threshold
              depth,                                    ///< 深度切断值. 这个值是考虑到过远的深度值测量不准确, 误差较大, 所以直接将距离相机过远的深度值"切掉",我们直接不用了
              icp,                                      ///< Relative ICP/RGB tracking weight
              icpErrThresh,                             ///< Local loop closure residual threshold
              covThresh,                                ///< Local loop closure covariance threshold
              photoThresh,                              ///< Global loop closure photometric threshold
              fernThresh;                               ///< Fern encoding threshold

        int timeDelta,                                  ///< Time window length
            icpCountThresh,                             ///< Local loop closure inlier threshold
            start,                                      ///< Frames to skip at start of log 跳过头几帧， 存储的数字为帧id
            end;                                        ///< Cut off frame of log 执行多少帧， 存储的为最后一帧处理的id

        bool fillIn,
             openLoop,                                  ///< Open loop mode, 不使用闭环
             reloc,                                     ///< Enable relocalisation
             frameskip,                                 ///< Frame skip if processing a log to simulate real-time. 就是如果处理比较慢， 那么就根据时间戳跳帧
             quiet,                                     ///< Quit when finished a log 播放记录文件完成后自动退出
             fastOdom,                                  ///< Fast odometry (single level pyramid)
             so3,                                       ///< Disables SO(3) pre-alignment in tracking
             rewind,                                    ///< Rewind and loop log forever 设置数据记录文件是否需要反复循环读取
             frameToFrameRGB;                           ///< Do frame-to-frame RGB tracking

        int framesToSkip;                               ///< 为了仿真"实时处理数据", 当处理速度跟不上帧率的时候会产生的跳帧的个数
        bool streaming;
        bool resetButton;                               ///< 复位请求

        Resize * resizeStream;                          ///< 使用 Shader 对原始图像进行降采样的对象
};

#endif /* MAINCONTROLLER_H_ */
