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

        bool good;
        ElasticFusion * eFusion;
        GUI * gui;
        GroundTruthOdometry * groundTruthOdometry;
        LogReader * logReader;      ///< 记录文件读取器的句柄

        bool iclnuim;               ///< 命令行中是否指定了参数 -icl
        std::string logFile;        ///< 命令行中指定的记录文件路径
        std::string poseFile;

        float confidence,
              depth,
              icp,
              icpErrThresh,
              covThresh,
              photoThresh,
              fernThresh;

        int timeDelta,
            icpCountThresh,
            start,
            end;

        bool fillIn,
             openLoop,
             reloc,
             frameskip,
             quiet,
             fastOdom,
             so3,
             rewind,
             frameToFrameRGB;

        int framesToSkip;
        bool streaming;
        bool resetButton;

        Resize * resizeStream;
};

#endif /* MAINCONTROLLER_H_ */
