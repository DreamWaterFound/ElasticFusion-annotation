/**
 * @file MainController.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 实现 GUI 程序的主要功能
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
 
#include "MainController.h"


// 构造函数
MainController::MainController(int argc, char * argv[])
 : 
   // ? 下面的 这些目前还意义不明
   good(true),
   eFusion(0),
   gui(0),
   groundTruthOdometry(0),
   logReader(0),
   framesToSkip(0),
   resetButton(false),
   resizeStream(0)
{
    // step 0 解析命令行参数
    // 这个变量用于缓存后面多个参数的内容
    std::string empty;
    // 如果找到了, 返回值>=0; 反之找不到
    // ? 这个变量好像一直没有被用到?
    iclnuim = Parse::get().arg(argc, argv, "-icl", empty) > -1;

    // 解析相机参数文件
    std::string calibrationFile;
    Parse::get().arg(argc, argv, "-cal", calibrationFile);

    // 设定分辨率对象, 这个分辨率和 Kinect1 的分辨率是相同的
    Resolution::getInstance(640, 480);

    if(calibrationFile.length())
    {
        // 如果命令行参数中的确指定要使用的相机内参
        loadCalibration(calibrationFile);
    }
    else
    {
        // 如果命令行参数中没有指定相机内参文件, 那么就使用默认内参
        Intrinsics::getInstance(528, 528, 320, 240);
    }

    // step 1 查看是否给定要使用录制的文件, 从而决定使用已经录制的文件还是使用实时的相机作为数据源
    Parse::get().arg(argc, argv, "-l", logFile);

    if(logFile.length())
    {
        // 如果要使用录制的文件, 就要生成记录文件读取器
        logReader = new RawLogReader(
            logFile,                                            // 记录文件的路径
            Parse::get().arg(argc, argv, "-f", empty) > -1);    // 是否要左右翻转得到的图像, 这里只需要知道是否存在这个参数标志就可以了
    }
    else
    {
        // 如果使用实际的传感器, 那么就生成所谓的"实时记录读取器", 其实就是获取相机实时图像的接口
        bool flipColors = Parse::get().arg(argc,argv,"-f",empty) > -1;
        logReader = new LiveLogReader(
            logFile,                // 此时这个记录文件路径为空啊, 并没有起到什么作用
            flipColors,             // 是否翻转图像
            LiveLogReader::CameraType::OpenNI2); // 使用的相机类型

        // 是否已经正确初始化数据源
        // ! 其实这里还有一种情况是 cam 为 nullptr, 下面的这个操作可能会触发段错误
        good = ((LiveLogReader *)logReader)->cam->ok();

        // 如果要使用Realsense作输入
#ifdef WITH_REALSENSE
        if(!good)
        {
          delete logReader;
          logReader = new LiveLogReader(logFile, flipColors, LiveLogReader::CameraType::RealSense);

          good = ((LiveLogReader *)logReader)->cam->ok();
        }
#endif
    }

    // step 2 查看是否给出了真值文件
    if(Parse::get().arg(argc, argv, "-p", poseFile) > 0)
    {
        // 那么就从外部文件中加载相机的运动轨迹
        groundTruthOdometry = new GroundTruthOdometry(poseFile);
    }

    // step 3 初始化一些参数
    confidence      = 10.0f;                //? 需要完善补充注释
    depth           = 3.0f;
    icp             = 10.0f;
    icpErrThresh    = 5e-05;
    covThresh       = 1e-05;
    photoThresh     = 115;
    fernThresh      = 0.3095f;

    timeDelta       = 200;
    icpCountThresh  = 40000;
    start           = 1;

    // 是否命令行参数中指定了 -nso , 如果指定了结果为 false
    so3 = !(Parse::get().arg(argc, argv, "-nso", empty) > -1);
    // ? 什么的边界? 帧数的？ 
    end = std::numeric_limits<unsigned short>::max(); // Funny bound, since we predict times in this format really!

    // 从命令行参数中更新这些数值
    Parse::get().arg(argc, argv, "-c", confidence);
    Parse::get().arg(argc, argv, "-d", depth);              // 深度切断值
    Parse::get().arg(argc, argv, "-i", icp);
    Parse::get().arg(argc, argv, "-ie", icpErrThresh);
    Parse::get().arg(argc, argv, "-cv", covThresh);
    Parse::get().arg(argc, argv, "-pt", photoThresh);
    Parse::get().arg(argc, argv, "-ft", fernThresh);
    Parse::get().arg(argc, argv, "-t", timeDelta);
    Parse::get().arg(argc, argv, "-ic", icpCountThresh);
    Parse::get().arg(argc, argv, "-s", start);
    Parse::get().arg(argc, argv, "-e", end);                // ? 指定处理的帧数最大值?

    // 从命令行参数得到是否需要对图像进行翻转
    logReader->flipColors = Parse::get().arg(argc, argv, "-f", empty) > -1;

    
    // ! 有个问题, 如果 groundTruthOdometry 并未初始化, 则这个指针可能不是 nullptr
    openLoop        = !groundTruthOdometry && Parse::get().arg(argc, argv, "-o", empty) > -1;       //? 是否开环
    reloc           = Parse::get().arg(argc, argv, "-rl", empty) > -1;                              //? 是否重定位
    frameskip       = Parse::get().arg(argc, argv, "-fs", empty) > -1;                              //? 是否跳帧
    quiet           = Parse::get().arg(argc, argv, "-q", empty) > -1;                               //? 安静模式? 数据文件读取完成之后直接退出?
    fastOdom        = Parse::get().arg(argc, argv, "-fo", empty) > -1;                              // 纯里程计模式
    rewind          = Parse::get().arg(argc, argv, "-r", empty) > -1;                               // 对于数据记录文件是否循环读取(播放)
    frameToFrameRGB = Parse::get().arg(argc, argv, "-ftf", empty) > -1;                             // Frame to frame 的工作方式

    // step 4 初始化 GUI 界面
    gui = new GUI(
        logFile.length() == 0,                              // 是否指定了记录文件路径, 没有指定则为 true
        Parse::get().arg(argc, argv, "-sc", empty) > -1);   // MapViewer部分全屏(即全屏展示构建的模型, 完全没有其他的东西)

    // 设置 Viewer 中的一些变量
    gui->flipColors->Ref().Set(logReader->flipColors);      // 彩色通道翻转
    gui->rgbOnly->Ref().Set(false);                         // ? 仅使用rgb?
    gui->pyramid->Ref().Set(true);                          // 使用图像金字塔
    gui->fastOdom->Ref().Set(fastOdom);                     // 使用快速视觉里程计模式(不建图)
    gui->confidenceThreshold->Ref().Set(confidence);        // ?
    gui->depthCutoff->Ref().Set(depth);                     // 深度切断\值
    gui->icpWeight->Ref().Set(icp);                         // ?
    gui->so3->Ref().Set(so3);                               // ?
    gui->frameToFrameRGB->Ref().Set(frameToFrameRGB);       // ? 使用帧-帧RGB信息估计相机位姿?

    // 生成用于将图像从原始图像大小降采样一般的图像缩放对象
    resizeStream = new Resize(Resolution::getInstance().width(),
                              Resolution::getInstance().height(),
                              Resolution::getInstance().width() / 2,
                              Resolution::getInstance().height() / 2);
}

// 析构函数, 就是释放各个 new 出来的资源
MainController::~MainController()
{
    if(eFusion)
    {
        delete eFusion;
    }

    if(gui)
    {
        delete gui;
    }

    if(groundTruthOdometry)
    {
        delete groundTruthOdometry;
    }

    if(logReader)
    {
        delete logReader;
    }

    if(resizeStream)
    {
        delete resizeStream;
    }
}

// 加载指定文件中的相机内参信息
void MainController::loadCalibration(const std::string & filename)
{
    // 输入文件流对象
    std::ifstream file(filename);
    // 保存每一行的信息
    std::string line;

    // 确保不是空文件
    assert(!file.eof());

    double fx, fy, cx, cy;

    // 读取内参
    std::getline(file, line);

    int n = sscanf(line.c_str(), "%lg %lg %lg %lg", &fx, &fy, &cx, &cy);

    // 严格检查
    assert(n == 4 && "Ooops, your calibration file should contain a single line with fx fy cx cy!");

    // 构造内参对象 静态类对象, 生成一次, 到处均可访问
    Intrinsics::getInstance(fx, fy, cx, cy);
}

// 运行
void MainController::launch()
{
    // 只在前面的准备工作都成功的时候才会执行.
    while(good)
    {
        // 如果已经创建了 ElasticFusion 对象， 那么那就正常运行吧
        if(eFusion)
        {
            run();
        }

        // 如果没有创建 ElasticFusion 对象， 或者是复位按钮被按下
        if(eFusion == 0 || resetButton)
        {
            // 复位标志复位
            resetButton = false;

            // 如果已经构造了 efusion, 那么我们就删除再重新建立它
            if(eFusion)
            {
                delete eFusion;
            }

            // 重新读取记录文件, 如果是跑实时的摄像头呢下面的函数是空的
            logReader->rewind();
            // 重新构建 ElasticFusion
            eFusion = new ElasticFusion(
                openLoop ? std::numeric_limits<int>::max() / 2 : timeDelta,     // ? 下面的都需要补充
                icpCountThresh,
                icpErrThresh,                                                   // ? ICP 迭代误差的最小值的阈值?
                covThresh,
                !openLoop,
                iclnuim,                                                        // 命令行中是否指定了参数 -icl
                reloc,                                                          // ? 是否使能了重定位
                photoThresh,
                confidence,
                depth,                                                          // 深度切断值
                icp,
                fastOdom,                                                       // 是否工作于纯里程计模式
                fernThresh,
                so3,
                frameToFrameRGB,
                logReader->getFile());                                          // 记录文件的位置, 对于实时摄像头来说是 basedir/live
        }
        else
        {
            // 运行到这里, 说明 eFusion!=0 并且 resetButton = false. 只能够说明是用户退出了
            break;
        }

    }
}

// 主循环, 实现给 ElasticFusion Core 喂图像的工作
void MainController::run()
{
    // Main loop
    while(!pangolin::ShouldQuit()                       // 终止条件1: Pangolin 窗口收到了退出信息 (比如按下ESC)
          && !((!logReader->hasMore()) && quiet)        // 终止条件2: 如果 quiet 有效, 并且记录文件已经读取完成了 // ? 但是这里的quiet是干嘛的? 
          && !(eFusion->getTick() == end && quiet))     // 终止条件3: 如果 quiet 有效, 并且也在给定的处理的图像帧数范围内 // ?
    {
        
        if(!gui->pause->Get() || pangolin::Pushed(*gui->step))
        {
            // 如果没有暂停, 并且也没有处于步进模式的话
            if((logReader->hasMore() || rewind)         // 喂图像条件1: 如果记录文件中还有数据(对live的相机来说一直成立), 如果没有数据的话, 之前设置了数据记录文件循环读取也行
                && eFusion->getTick() < end)            // 喂图像条件2: 如果 ElasticFusion 处理的图像数量在预想的范围内
            {
                // 开始统计 日志文件读取(LogRead) 所耗费的时间
                TICK("LogRead");

                // HERE
                if(rewind)
                {
                    // 如果设置了数据记录文件要反复读取
                    if(!logReader->hasMore())
                    {
                        logReader->getBack();
                    }
                    else
                    {
                        logReader->getNext();
                    }

                    if(logReader->rewound())
                    {
                        logReader->currentFrame = 0;
                    }
                }
                else
                {
                    logReader->getNext();
                }
                TOCK("LogRead");

                if(eFusion->getTick() < start)
                {
                    eFusion->setTick(start);
                    logReader->fastForward(start);
                }

                float weightMultiplier = framesToSkip + 1;

                if(framesToSkip > 0)
                {
                    eFusion->setTick(eFusion->getTick() + framesToSkip);
                    logReader->fastForward(logReader->currentFrame + framesToSkip);
                    framesToSkip = 0;
                }

                Eigen::Matrix4f * currentPose = 0;

                if(groundTruthOdometry)
                {
                    currentPose = new Eigen::Matrix4f;
                    currentPose->setIdentity();
                    *currentPose = groundTruthOdometry->getTransformation(logReader->timestamp);
                }

                eFusion->processFrame(logReader->rgb, logReader->depth, logReader->timestamp, currentPose, weightMultiplier);

                if(currentPose)
                {
                    delete currentPose;
                }

                if(frameskip && Stopwatch::getInstance().getTimings().at("Run") > 1000.f / 30.f)
                {
                    framesToSkip = int(Stopwatch::getInstance().getTimings().at("Run") / (1000.f / 30.f));
                }
            }
        }
        else
        {
            // 如果处于暂停或者是步进模式下, 那么就只 predict 模型的图像
            eFusion->predict();
        }


        // TODO 看上去和调试有关
        TICK("GUI");

        
        if(gui->followPose->Get())
        {
            // 如果选择了使用相机当前的位姿进行观测
            pangolin::OpenGlMatrix mv;

            Eigen::Matrix4f currPose = eFusion->getCurrPose();
            Eigen::Matrix3f currRot = currPose.topLeftCorner(3, 3);

            Eigen::Quaternionf currQuat(currRot);
            Eigen::Vector3f forwardVector(0, 0, 1);
            Eigen::Vector3f upVector(0, iclnuim ? 1 : -1, 0);

            Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
            Eigen::Vector3f up = (currQuat * upVector).normalized();

            Eigen::Vector3f eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));

            eye -= forward;

            Eigen::Vector3f at = eye + forward;

            Eigen::Vector3f z = (eye - at).normalized();  // Forward
            Eigen::Vector3f x = up.cross(z).normalized(); // Right
            Eigen::Vector3f y = z.cross(x);

            Eigen::Matrix4d m;
            m << x(0),  x(1),  x(2),  -(x.dot(eye)),
                 y(0),  y(1),  y(2),  -(y.dot(eye)),
                 z(0),  z(1),  z(2),  -(z.dot(eye)),
                    0,     0,     0,              1;

            memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

            gui->s_cam.SetModelViewMatrix(mv);
        }

        // 进行绘制 MapViewer 的准备工作, 主要是清空各种缓冲区, 并且激活模型的绘制区域
        gui->preCall();

        std::stringstream stri;
        stri << eFusion->getModelToModel().lastICPCount;
        gui->trackInliers->Ref().Set(stri.str());

        std::stringstream stre;
        stre << (std::isnan(eFusion->getModelToModel().lastICPError) ? 0 : eFusion->getModelToModel().lastICPError);
        gui->trackRes->Ref().Set(stre.str());

        if(!gui->pause->Get())
        {
            gui->resLog.Log((std::isnan(eFusion->getModelToModel().lastICPError) ? std::numeric_limits<float>::max() : eFusion->getModelToModel().lastICPError), icpErrThresh);
            gui->inLog.Log(eFusion->getModelToModel().lastICPCount, icpCountThresh);
        }

        Eigen::Matrix4f pose = eFusion->getCurrPose();

        if(gui->drawRawCloud->Get() || gui->drawFilteredCloud->Get())
        {
            eFusion->computeFeedbackBuffers();
        }

        if(gui->drawRawCloud->Get())
        {
            eFusion->getFeedbackBuffers().at(FeedbackBuffer::RAW)->render(gui->s_cam.GetProjectionModelViewMatrix(), pose, gui->drawNormals->Get(), gui->drawColors->Get());
        }

        if(gui->drawFilteredCloud->Get())
        {
            eFusion->getFeedbackBuffers().at(FeedbackBuffer::FILTERED)->render(gui->s_cam.GetProjectionModelViewMatrix(), pose, gui->drawNormals->Get(), gui->drawColors->Get());
        }

        if(gui->drawGlobalModel->Get())
        {
            glFinish();
            TICK("Global");

            if(gui->drawFxaa->Get())
            {
                gui->drawFXAA(gui->s_cam.GetProjectionModelViewMatrix(),
                              gui->s_cam.GetModelViewMatrix(),
                              eFusion->getGlobalModel().model(),
                              eFusion->getConfidenceThreshold(),
                              eFusion->getTick(),
                              eFusion->getTimeDelta(),
                              iclnuim);
            }
            else
            {
                eFusion->getGlobalModel().renderPointCloud(gui->s_cam.GetProjectionModelViewMatrix(),
                                                           eFusion->getConfidenceThreshold(),
                                                           gui->drawUnstable->Get(),
                                                           gui->drawNormals->Get(),
                                                           gui->drawColors->Get(),
                                                           gui->drawPoints->Get(),
                                                           gui->drawWindow->Get(),
                                                           gui->drawTimes->Get(),
                                                           eFusion->getTick(),
                                                           eFusion->getTimeDelta());
            }
            glFinish();
            TOCK("Global");
        }

        if(eFusion->getLost())
        {
            glColor3f(1, 1, 0);
        }
        else
        {
            glColor3f(1, 0, 1);
        }
        gui->drawFrustum(pose);
        glColor3f(1, 1, 1);

        if(gui->drawFerns->Get())
        {
            glColor3f(0, 0, 0);
            for(size_t i = 0; i < eFusion->getFerns().frames.size(); i++)
            {
                if((int)i == eFusion->getFerns().lastClosest)
                    continue;

                gui->drawFrustum(eFusion->getFerns().frames.at(i)->pose);
            }
            glColor3f(1, 1, 1);
        }

        if(gui->drawDefGraph->Get())
        {
            const std::vector<GraphNode*> & graph = eFusion->getLocalDeformation().getGraph();

            for(size_t i = 0; i < graph.size(); i++)
            {
                pangolin::glDrawCross(graph.at(i)->position(0),
                                      graph.at(i)->position(1),
                                      graph.at(i)->position(2),
                                      0.1);

                for(size_t j = 0; j < graph.at(i)->neighbours.size(); j++)
                {
                    pangolin::glDrawLine(graph.at(i)->position(0),
                                         graph.at(i)->position(1),
                                         graph.at(i)->position(2),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(0),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(1),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(2));
                }
            }
        }

        if(eFusion->getFerns().lastClosest != -1)
        {
            glColor3f(1, 0, 0);
            gui->drawFrustum(eFusion->getFerns().frames.at(eFusion->getFerns().lastClosest)->pose);
            glColor3f(1, 1, 1);
        }

        const std::vector<PoseMatch> & poseMatches = eFusion->getPoseMatches();

        int maxDiff = 0;
        for(size_t i = 0; i < poseMatches.size(); i++)
        {
            if(poseMatches.at(i).secondId - poseMatches.at(i).firstId > maxDiff)
            {
                maxDiff = poseMatches.at(i).secondId - poseMatches.at(i).firstId;
            }
        }

        for(size_t i = 0; i < poseMatches.size(); i++)
        {
            if(gui->drawDeforms->Get())
            {
                if(poseMatches.at(i).fern)
                {
                    glColor3f(1, 0, 0);
                }
                else
                {
                    glColor3f(0, 1, 0);
                }
                for(size_t j = 0; j < poseMatches.at(i).constraints.size(); j++)
                {
                    pangolin::glDrawLine(poseMatches.at(i).constraints.at(j).sourcePoint(0), poseMatches.at(i).constraints.at(j).sourcePoint(1), poseMatches.at(i).constraints.at(j).sourcePoint(2),
                                         poseMatches.at(i).constraints.at(j).targetPoint(0), poseMatches.at(i).constraints.at(j).targetPoint(1), poseMatches.at(i).constraints.at(j).targetPoint(2));
                }
            }
        }
        glColor3f(1, 1, 1);

        eFusion->normaliseDepth(0.3f, gui->depthCutoff->Get());

        for(std::map<std::string, GPUTexture*>::const_iterator it = eFusion->getTextures().begin(); it != eFusion->getTextures().end(); ++it)
        {
            if(it->second->draw)
            {
                gui->displayImg(it->first, it->second);
            }
        }

        eFusion->getIndexMap().renderDepth(gui->depthCutoff->Get());

        gui->displayImg("ModelImg", eFusion->getIndexMap().imageTex());
        gui->displayImg("Model", eFusion->getIndexMap().drawTex());

        std::stringstream strs;
        strs << eFusion->getGlobalModel().lastCount();

        gui->totalPoints->operator=(strs.str());

        std::stringstream strs2;
        strs2 << eFusion->getLocalDeformation().getGraph().size();

        gui->totalNodes->operator=(strs2.str());

        std::stringstream strs3;
        strs3 << eFusion->getFerns().frames.size();

        gui->totalFerns->operator=(strs3.str());

        std::stringstream strs4;
        strs4 << eFusion->getDeforms();

        gui->totalDefs->operator=(strs4.str());

        std::stringstream strs5;
        strs5 << eFusion->getTick() << "/" << logReader->getNumFrames();

        gui->logProgress->operator=(strs5.str());

        std::stringstream strs6;
        strs6 << eFusion->getFernDeforms();

        gui->totalFernDefs->operator=(strs6.str());

        gui->postCall();

        logReader->flipColors = gui->flipColors->Get();
        eFusion->setRgbOnly(gui->rgbOnly->Get());
        eFusion->setPyramid(gui->pyramid->Get());
        eFusion->setFastOdom(gui->fastOdom->Get());
        eFusion->setConfidenceThreshold(gui->confidenceThreshold->Get());
        eFusion->setDepthCutoff(gui->depthCutoff->Get());
        eFusion->setIcpWeight(gui->icpWeight->Get());
        eFusion->setSo3(gui->so3->Get());
        eFusion->setFrameToFrameRGB(gui->frameToFrameRGB->Get());

        resetButton = pangolin::Pushed(*gui->reset);

        if(gui->autoSettings)
        {
            static bool last = gui->autoSettings->Get();

            if(gui->autoSettings->Get() != last)
            {
                last = gui->autoSettings->Get();
                static_cast<LiveLogReader *>(logReader)->setAuto(last);
            }
        }

        Stopwatch::getInstance().sendAll();

        if(resetButton)
        {
            break;
        }

        if(pangolin::Pushed(*gui->save))
        {
            eFusion->savePly();
        }

        TOCK("GUI");
    }
}
