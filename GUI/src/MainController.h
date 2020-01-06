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

#include <ElasticFusion.h>
#include <Utils/Parse.h>

#include "Tools/GUI.h"
#include "Tools/GroundTruthOdometry.h"
#include "Tools/RawLogReader.h"
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

        // ? 还有向下兼容的可能?
        virtual ~MainController();

        /** @brief 运行 */
        void launch();

    private:
        void run();

        /**
         * @brief 加载指定文件中的相机内参信息
         * @param[in] filename 文件名
         */
        void loadCalibration(const std::string & filename);

        bool good;                  ///< 当前是否已经正确地初始化数据源了
        ElasticFusion * eFusion;
        GUI * gui;                  ///< GUI 窗口对象
        GroundTruthOdometry * groundTruthOdometry;      ///< 轨迹真值文件对象句柄
        LogReader * logReader;      ///< 记录文件读取器的句柄, 根据情况也会是实时相机的接口

        bool iclnuim;               ///< 命令行中是否指定了参数 -icl
        std::string logFile;        ///< 命令行中指定的记录文件路径
        std::string poseFile;       ///< 命令行参数中指定的真值文件

        float confidence,           ///? 什么的置信度?
              depth,                ///< 深度切断值
              icp,                  ///?
              icpErrThresh,         ///? ICP 迭代误差的最小阈值?
              covThresh,            ///?
              photoThresh,          ///? 光度误差的最小阈值?
              fernThresh;           ///? 随机蕨?

        int timeDelta,              ///?
            icpCountThresh,         ///?
            start,                  ///?
            end;                    ///?

        bool fillIn,
             openLoop,              ///? 是否以开环模式运行
             reloc,                 ///? 是否使能重定位
             frameskip,             ///? 跳帧?
             quiet,                 ///< 是否安静模式
             fastOdom,              ///< 是否纯里程计模式
             so3,                   ///? 和命令行参数中是否指定 -nso 参数有关系
             rewind,                ///?
             frameToFrameRGB;       ///? 是否使用 Frame to Frame 工作模式

        int framesToSkip;
        bool streaming;
        bool resetButton;

        Resize * resizeStream;
};

#endif /* MAINCONTROLLER_H_ */
